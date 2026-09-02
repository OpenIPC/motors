#include "pelcodtui.h"
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static void section_name(char *out,size_t n,const char *p,const char *s){snprintf(out,n,"[%s.%s]",p,s);}
static void copy_value(char *out,size_t n,const char *value){if(!n)return;size_t len=strlen(value);if(len>=n)len=n-1;memcpy(out,value,len);out[len]=0;}
int pct_state_read(const char *path,const char *profile,const char *setting,struct pct_state_record *out){
    memset(out,0,sizeof(*out));FILE*f=fopen(path,"r");if(!f)return-1;char target[200],line[1024];section_name(target,sizeof(target),profile,setting);bool in=false;
    while(fgets(line,sizeof(line),f)){line[strcspn(line,"\r\n")]=0;if(line[0]=='[')in=!strcmp(line,target);else if(in){if(!strncmp(line,"value=",6))copy_value(out->value,sizeof(out->value),line+6);else if(!strncmp(line,"command=",8))copy_value(out->command,sizeof(out->command),line+8);else if(!strncmp(line,"timestamp=",10))copy_value(out->timestamp,sizeof(out->timestamp),line+10);}}
    fclose(f);return out->timestamp[0]?0:-1;
}
int pct_state_write(const char *path,const char *profile,const char *setting,const char *value,const char *command,char *err,size_t n){
    char tmp[300];snprintf(tmp,sizeof(tmp),"%s.tmp",path);FILE*out=fopen(tmp,"w");if(!out){snprintf(err,n,"state: %s",strerror(errno));return-1;}FILE*in=fopen(path,"r");char target[200],line[1024];section_name(target,sizeof(target),profile,setting);bool skip=false;
    if(in){while(fgets(line,sizeof(line),in)){if(line[0]=='[')skip=!strncmp(line,target,strlen(target))&&(line[strlen(target)]=='\n'||line[strlen(target)]=='\r'||line[strlen(target)]==0);if(!skip)fputs(line,out);}fclose(in);}
    time_t now=time(NULL);struct tm tm;localtime_r(&now,&tm);char stamp[40];strftime(stamp,sizeof(stamp),"%Y-%m-%d %H:%M:%S %z",&tm);
    fprintf(out,"\n%s\nvalue=%s\ncommand=%s\ntimestamp=%s\n",target,value,command,stamp);fflush(out);fsync(fileno(out));if(fclose(out)||rename(tmp,path)){snprintf(err,n,"state save: %s",strerror(errno));unlink(tmp);return-1;}return 0;
}

int pct_state_remove(const char *path, const char *profile, const char *setting,
                     char *err, size_t n) {
    char tmp[300], target[200], line[1024];
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    FILE *in = fopen(path, "r");
    if (!in) {
        if (errno == ENOENT) return 0;
        snprintf(err, n, "state: %s", strerror(errno));
        return -1;
    }
    FILE *out = fopen(tmp, "w");
    if (!out) {
        snprintf(err, n, "state: %s", strerror(errno));
        fclose(in);
        return -1;
    }
    section_name(target, sizeof(target), profile, setting);
    bool skip = false;
    while (fgets(line, sizeof(line), in)) {
        if (line[0] == '[')
            skip = !strncmp(line, target, strlen(target)) &&
                   (line[strlen(target)] == '\n' ||
                    line[strlen(target)] == '\r' ||
                    line[strlen(target)] == 0);
        if (!skip) fputs(line, out);
    }
    fclose(in);
    fflush(out);
    fsync(fileno(out));
    if (fclose(out) || rename(tmp, path)) {
        snprintf(err, n, "state save: %s", strerror(errno));
        unlink(tmp);
        return -1;
    }
    return 0;
}
