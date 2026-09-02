#include "pelcodtui.h"

#include <ctype.h>
#include <errno.h>
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
