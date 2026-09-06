#include "pelcodtui.h"

#include <ctype.h>
#include <errno.h>
#include <json-c/json.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char *trim(char *s) {
    while (*s && isspace((unsigned char)*s)) s++;
    char *e = s + strlen(s);
    while (e > s && isspace((unsigned char)e[-1])) --e;
    *e = 0;
    return s;
}

static int add(struct pct_profile *p, const char *sec, const char *key, const char *val) {
    if (p->count >= PCT_MAX_ENTRIES) return -1;
    snprintf(p->entries[p->count].section, 96, "%s", sec);
    snprintf(p->entries[p->count].key, 96, "%s", key);
    snprintf(p->entries[p->count].value, PCT_TEXT, "%s", val);
    p->count++;
    return 0;
}

static int add_section(struct pct_profile *p, const char *section) {
    size_t length = strlen(section);
    if (length >= sizeof(p->sections[0])) return -1;
    for (size_t i = 0; i < p->section_count; ++i)
        if (!strcmp(p->sections[i], section)) return 0;
    if (p->section_count >= PCT_MAX_SECTIONS) return -1;
    memcpy(p->sections[p->section_count], section, length + 1);
    p->section_count++;
    return 0;
}

static int add_json_string(struct pct_profile *p, const char *section,
                           struct json_object *object, const char *key) {
    struct json_object *value = NULL;
    if (!json_object_object_get_ex(object, key, &value)) return 0;
    return add(p, section, key, json_object_get_string(value));
}

static int add_json_array(struct pct_profile *p, const char *section,
                          struct json_object *object, const char *key,
                          const char *value_key) {
    struct json_object *array = NULL;
    if (!json_object_object_get_ex(object, key, &array) ||
        !json_object_is_type(array, json_type_array)) return 0;
    char joined[PCT_TEXT] = "";
    size_t used = 0;
    for (size_t i = 0; i < json_object_array_length(array); ++i) {
        struct json_object *item = json_object_array_get_idx(array, i);
        struct json_object *value = item;
        if (value_key && (!json_object_object_get_ex(item, value_key, &value)))
            return -1;
        const char *text = json_object_get_string(value);
        int written = snprintf(joined + used, sizeof(joined) - used, "%s%s",
                               used ? "," : "", text ? text : "");
        if (written < 0 || (size_t)written >= sizeof(joined) - used) return -1;
        used += (size_t)written;
    }
    return add(p, section, key, joined);
}

int pct_profile_from_description(struct pct_profile *p, const char *json,
                                 char *err, size_t n) {
    memset(p, 0, sizeof(*p));
    struct json_object *root = json_tokener_parse(json);
    struct json_object *ok = NULL, *profile = NULL, *label = NULL;
    struct json_object *menus = NULL, *controls = NULL;
    if (!root || !json_object_is_type(root, json_type_object) ||
        !json_object_object_get_ex(root, "ok", &ok) ||
        !json_object_get_boolean(ok) ||
        !json_object_object_get_ex(root, "profile", &profile) ||
        !json_object_object_get_ex(root, "label", &label) ||
        !json_object_object_get_ex(root, "menus", &menus) ||
        !json_object_is_type(menus, json_type_array) ||
        !json_object_object_get_ex(root, "controls", &controls) ||
        !json_object_is_type(controls, json_type_array)) {
        snprintf(err, n, "invalid driver description");
        if (root) json_object_put(root);
        return -1;
    }
    add_section(p, "profile");
    if (add(p, "profile", "schema_version", "1") ||
        add(p, "profile", "id", json_object_get_string(profile)) ||
        add(p, "profile", "name", json_object_get_string(label)) ||
        add_json_string(p, "profile", root, "description") ||
        add_json_string(p, "profile", root, "dangerous_call") ||
        add_json_string(p, "profile", root, "dangerous_set")) goto too_large;

    for (size_t i = 0; i < json_object_array_length(menus); ++i) {
        struct json_object *menu = json_object_array_get_idx(menus, i), *id = NULL;
        char section[128];
        if (!json_object_object_get_ex(menu, "id", &id)) goto invalid;
        snprintf(section, sizeof(section), "menu.%s", json_object_get_string(id));
        if (add_section(p, section) || add_json_string(p, section, menu, "label") ||
            add_json_array(p, section, menu, "items", NULL)) goto too_large;
    }
    for (size_t i = 0; i < json_object_array_length(controls); ++i) {
        struct json_object *control = json_object_array_get_idx(controls, i);
        struct json_object *id = NULL, *type = NULL;
        if (!json_object_object_get_ex(control, "id", &id) ||
            !json_object_object_get_ex(control, "type", &type)) goto invalid;
        bool action = !strcmp(json_object_get_string(type), "action");
        char section[128];
        snprintf(section, sizeof(section), "%s.%s", action ? "action" : "setting",
                 json_object_get_string(id));
        if (add_section(p, section)) goto too_large;
        const char *keys[] = {"type", "label", "description", "default", "unit",
                              "warning", "min", "max", "step", "confirm"};
        for (size_t k = 0; k < sizeof(keys) / sizeof(keys[0]); ++k)
            if ((!action || strcmp(keys[k], "type")) &&
                add_json_string(p, section, control, keys[k])) goto too_large;
        struct json_object *options = NULL;
        if (json_object_object_get_ex(control, "options", &options)) {
            if (add_json_array(p, section, control, "options", "value")) goto too_large;
            for (size_t j = 0; j < json_object_array_length(options); ++j) {
                struct json_object *option = json_object_array_get_idx(options, j);
                struct json_object *value = NULL, *option_label = NULL;
                if (!json_object_object_get_ex(option, "value", &value)) goto invalid;
                char key[192];
                snprintf(key, sizeof(key), "option.%s.label", json_object_get_string(value));
                if (json_object_object_get_ex(option, "label", &option_label) &&
                    add(p, section, key, json_object_get_string(option_label))) goto too_large;
            }
        }
    }
    json_object_put(root);
    return 0;
too_large:
    snprintf(err, n, "driver description is too large");
    json_object_put(root);
    return -1;
invalid:
    snprintf(err, n, "invalid driver description");
    json_object_put(root);
    return -1;
}

const char *pct_get(const struct pct_profile *p, const char *section, const char *key) {
    for (size_t i = 0; i < p->count; i++)
        if (!strcmp(p->entries[i].section, section) && !strcmp(p->entries[i].key, key))
            return p->entries[i].value;
    return NULL;
}

bool pct_csv_has(const char *csv, const char *value) {
    if (!csv || !value) return false;
    char buf[PCT_TEXT]; snprintf(buf, sizeof(buf), "%s", csv);
    char *save = NULL;
    for (char *s = strtok_r(buf, ",", &save); s; s = strtok_r(NULL, ",", &save))
        if (!strcmp(trim(s), value)) return true;
    return false;
}

int pct_profile_load(struct pct_profile *p, const char *path, char *err, size_t n) {
    memset(p, 0, sizeof(*p));
    FILE *f = fopen(path, "r");
    if (!f) { snprintf(err, n, "open %s: %s", path, strerror(errno)); return -1; }
    snprintf(p->path, sizeof(p->path), "%s", path);
    char line[1024], sec[96] = "";
    while (fgets(line, sizeof(line), f)) {
        char *s = trim(line), *hash;
        if (!*s || *s == '#') continue;
        if ((hash = strchr(s, '#'))) *hash = 0;
        s = trim(s); if (!*s) continue;
        if (*s == '[') {
            char *e = strchr(s, ']');
            if (!e) { snprintf(err, n, "invalid section"); fclose(f); return -1; }
            *e = 0; snprintf(sec, sizeof(sec), "%s", s + 1);
            bool seen = false;
            for (size_t i=0;i<p->section_count;i++) if (!strcmp(p->sections[i],sec)) seen=true;
            if (!seen && p->section_count < PCT_MAX_SECTIONS)
                snprintf(p->sections[p->section_count++], 96, "%s", sec);
            continue;
        }
        char *eq = strchr(s, '=');
        if (!eq || !*sec) { snprintf(err, n, "key outside section"); fclose(f); return -1; }
        *eq = 0;
        if (add(p, sec, trim(s), trim(eq+1))) { snprintf(err,n,"profile too large"); fclose(f); return -1; }
    }
    fclose(f);
    return pct_profile_validate(p, err, n);
}

static bool uint_in(const char *s, unsigned lo, unsigned hi) {
    if (!s || !*s) return false;
    char *e; unsigned long v = strtoul(s, &e, 10);
    return *e == 0 && v >= lo && v <= hi;
}

static bool baud_valid(const char *s) {
    static const char *values[] = {"1200","2400","4800","9600","19200","38400","57600","115200"};
    if (!s) return false;
    for (size_t i=0;i<sizeof(values)/sizeof(values[0]);i++) if(!strcmp(s,values[i])) return true;
    return false;
}

static bool command_valid(const char *cmd, bool allow_value) {
    if (!cmd || !*cmd) return false;
    char buf[PCT_TEXT]; snprintf(buf,sizeof(buf),"%s",cmd);
    char *save=NULL;
    bool parsed=false;
    for(char *part=strtok_r(buf,";",&save);part;part=strtok_r(NULL,";",&save)) {
        parsed=true;
        struct pct_command command;
        if (pct_parse_command(trim(part), allow_value, &command)) return false;
    }
    return parsed;
}

int pct_profile_validate(const struct pct_profile *p, char *err, size_t n) {
    if (!pct_get(p,"profile","id") || strcmp(pct_get(p,"profile","schema_version") ?: "","1")) {
        snprintf(err,n,"profile requires schema_version=1 and id"); return -1;
    }
    const char *baud=pct_get(p,"uart","baud"),*address=pct_get(p,"uart","address");
    if(baud&&!baud_valid(baud)){snprintf(err,n,"uart invalid baud");return -1;}
    if(address&&!uint_in(address,1,255)){snprintf(err,n,"uart address must be 1..255");return -1;}
    for (size_t i=0;i<p->section_count;i++) {
        const char *s=p->sections[i];
        if (!strncmp(s,"menu.",5)) {
            const char *items=pct_get(p,s,"items");
            if(!items){snprintf(err,n,"%s missing items",s);return -1;}
            char b[PCT_TEXT];snprintf(b,sizeof(b),"%s",items);char *sv=NULL;
            for(char *id=strtok_r(b,",",&sv);id;id=strtok_r(NULL,",",&sv)){
                id=trim(id);char setting[128],action[128];snprintf(setting,sizeof(setting),"setting.%s",id);snprintf(action,sizeof(action),"action.%s",id);
                if(strcmp(id,"__preset_control")&&strcmp(id,"__tui_settings")&&!pct_get(p,setting,"label")&&!pct_get(p,action,"label")){snprintf(err,n,"%s unknown item %s",s,id);return -1;}
            }
        } else if (!strncmp(s,"setting.",8)) {
            const char *type=pct_get(p,s,"type"), *label=pct_get(p,s,"label"), *desc=pct_get(p,s,"description");
            if(!type||!label||!desc){snprintf(err,n,"%s missing metadata",s);return -1;}
            if(!strcmp(type,"number")) {
                const char *lo=pct_get(p,s,"min"),*hi=pct_get(p,s,"max"),*cmd=pct_get(p,s,"command");
                if(!uint_in(lo,0,255)||!uint_in(hi,1,255)||atoi(lo)>atoi(hi)||!command_valid(cmd,true)) {snprintf(err,n,"%s invalid number",s);return -1;}
            } else if(!strcmp(type,"choice")||!strcmp(type,"toggle")) {
                const char *opts=pct_get(p,s,"options"); if(!opts){snprintf(err,n,"%s missing options",s);return -1;}
                char b[PCT_TEXT];snprintf(b,sizeof(b),"%s",opts);char *sv=NULL;
                for(char *o=strtok_r(b,",",&sv);o;o=strtok_r(NULL,",",&sv)) {char k[160];snprintf(k,sizeof(k),"option.%s.command",trim(o));if(!command_valid(pct_get(p,s,k),false)){snprintf(err,n,"%s invalid option",s);return -1;}}
            } else {snprintf(err,n,"%s invalid type",s);return -1;}
        } else if (!strncmp(s,"action.",7) && !command_valid(pct_get(p,s,"command"),false)) {
            snprintf(err,n,"%s invalid action",s);return -1;
        }
    }
    return 0;
}

int pct_expand_setting(const struct pct_profile *p, const char *id, const char *value,
                       char *out, size_t n, char *err, size_t en) {
    char sec[128]; snprintf(sec,sizeof(sec),"setting.%s",id);
    const char *type=pct_get(p,sec,"type"), *cmd=NULL;
    if(!type){snprintf(err,en,"unknown setting");return -1;}
    if(!strcmp(type,"number")) {
        unsigned lo=atoi(pct_get(p,sec,"min")),hi=atoi(pct_get(p,sec,"max"));
        if(!uint_in(value,lo,hi)){snprintf(err,en,"value must be %u..%u",lo,hi);return -1;}
        cmd=pct_get(p,sec,"command");
    } else {
        if(!pct_csv_has(pct_get(p,sec,"options"),value)){snprintf(err,en,"invalid option");return -1;}
        char key[160];snprintf(key,sizeof(key),"option.%s.command",value);cmd=pct_get(p,sec,key);
    }
    const char *needle="$value", *q;
    size_t used=0,needle_len=strlen(needle),value_len=strlen(value);
    if(!n){snprintf(err,en,"expanded command too long");return -1;}
    while((q=strstr(cmd,needle))){
        size_t prefix=(size_t)(q-cmd);
        if(prefix+value_len>=n-used){snprintf(err,en,"expanded command too long");return -1;}
        memcpy(out+used,cmd,prefix);used+=prefix;
        memcpy(out+used,value,value_len);used+=value_len;
        cmd=q+needle_len;
    }
    size_t tail=strlen(cmd);
    if(tail>=n-used){snprintf(err,en,"expanded command too long");return -1;}
    memcpy(out+used,cmd,tail+1);return 0;
}
