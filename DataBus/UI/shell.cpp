#include <stdio.h>
#include "mbed.h"
#include "stdint.h"
#include "DirHandle.h"
#include "SDHCFileSystem.h"
#include "util.h"
#include "Buttons.h"

extern SDFileSystem sd;
extern Serial pc;
extern Buttons keypad;

char cwd[64];
char buf[128];
bool debug=false;

void shell(void);
void termInput(char *cmd);
void resolveDirectory(char *newpath, char *path);
void splitName(char *path, char *dirname, char *basename);
void ls(char *path);
void cd(char *path);
void pwd(void);
void touch(char *path);
void head(char *path);
void cat(char *path);
void send(char *path);

void shell() {
    FILE *fp;
    char newpath[64], *arg, cmd[64], cmdline[64];
    bool done=false;

    //Logger.SelectCRCMode(1);
    
    //pc.printf("Formatting...\n");
    //int i = Logger.format(32);
    //pc.printf("format result: %d\n", i);

    if ((fp = fopen("/log/message.txt", "w")) != NULL) {
        for (int i=0; i < 20; i++)
            fprintf(fp, "Hello, World!\n");
        fclose(fp);
        //pc.printf("created!\n");
    } else {
        pc.printf("Error creating file\n");
    }
    pc.printf("\n");
    
    strcpy(cwd, "/log");

    while (!done) {
        termInput(cmdline);
        arg = split(cmd, cmdline, 64, ' ');
        resolveDirectory(newpath, arg);
        
        if (keypad.pressed) {
            keypad.pressed = false;
            break;
        }        
        
        if (debug) pc.printf("cmdline:<%s> cmd:<%s> arg:<%s> newpath:<%s>\n", cmdline, cmd, arg, newpath);
        
        if (!strcmp(cmd, "ls")) {
            ls(newpath);
        } else if (!strcmp(cmd, "cd")) {
            cd(newpath);
        } else if (!strcmp(cmd, "pwd")) {
            pwd();
        } else if (!strcmp(cmd, "head")) {
            head(newpath);
        } else if (!strcmp(cmd, "cat")) {
            cat(newpath);
        } else if (!strcmp(cmd, "send")) {
            send(newpath);
        } else if (!strcmp(cmd, "mkdir")) {
            mkdir(newpath, 1023);
        } else if (!strcmp(cmd, "debug")) {
            debug = !debug;
        } else if (!strcmp(cmd, "touch")) {
            touch(newpath);
        } else if (!strcmp(cmd, "rm")) {
            remove(newpath);
        } else if (!strcmp(cmd, "exit")) {
            done = true;
        } else if (cmd[0] == '\0') {
            // ignore
        } else {
            pc.printf("%s: command not found\n", cmd);
        }
    }

/*
    pc.printf("Printing splitName()\n");
    splitName("/SDCard/testdir", dirname, basename);
    pc.printf("%s %s\n", dirname, basename);

    pc.printf("Printing resolveDirectory()\n");
    resolveDirectory(newpath, "test");
    pc.printf("%s\n", newpath);
*/

//    remove("/SDCard/testdir/TEST.txt");
    
    /*
    int test = rename("/SDCard/message.txt", "/SDCard/message2.txt");
    fp = fopen("/SDCard/message.txt", "a");
    fprintf(fp, "  Result = %d", test);
    fclose(fp);
    */

    pc.printf ("exiting shell\n");

    return;
}


/** termInput
 *
 */
void termInput(char *cmd) {
    int i=0;
    char c;
    bool done = false;
    
    memset(cmd, 0, 64);
    
    pc.printf("# ", cwd);
    do {
        cmd[i] = 0;
        c = pc.getc();
        if (c == '\r') { // if return is hit, we're done, don't add \r to cmd
            done = true;
        } else if (i < 64-1) {
            if (c == 0x7f || c == '\b') { // backspace or delete
                if (i > 0) { // if we're at the beginning, do nothing
                    i--;
                    pc.printf("\b \b");
                }
            } else {
                pc.printf("%c", c);
                cmd[i++] = c;
            }
        }
    } while (!done);
    pc.printf("\n");
} 

/** resolveDirectory
 * resolve the directory path provided, given the cwd
 */
void resolveDirectory(char *newpath, char *path) {
    char dirname[32], basename[16];
    
    /** absolute path */
    if (path[0] == '/') {
        strcpy(newpath, path);
    }
    /** relative path */
    else {
        strcpy(newpath, cwd);
        if (path[0] != 0) {
            if (newpath[strlen(newpath)-1] != '/')
                strcat(newpath, "/");
            strcat(newpath, path);    
        }
        /** Resolve .. references */
        splitName(newpath, dirname, basename);
        if (!strcmp(basename, "..")) {
            splitName(dirname, newpath, basename);
        }
    }
}

// copy t to s until space is reached
// return location of delimiter+1 in t
// if space not found, return t pointing to end of string
// if s or t null, return null
char *splitCmd(char *s, char *t, int max)
{
  int i = 0;
  
  if (s == 0 || t == 0)
    return 0;

  while (*t != 0 && i < max) {
    *s++ = *t++;
    i++;
    if (*t == ' ') {
        t++;
        break;
    }
  }
  *s = 0;
    
  return t;
}

/** splitName
 * split the path into a dirname and a 
 */
void splitName(char *path, char *dirname, char *basename) {
    int sep;
    
    sep = 0;
    if (debug) pc.printf("%d\n", strlen(path));
    for (int i=strlen(path)-1; i >= 0; i--) {
        if (debug) pc.printf("- %c\n", path[i]);
        sep = i;
        if (path[i] == '/') break;
    }
    for (int j=0; j < sep; j++) {
        if (debug) pc.printf("> %c\n", path[j]);
        dirname[j] = path[j];
        dirname[j+1] = 0;
    }
    for (int k=sep+1; k < strlen(path); k++) {
        if (debug) pc.printf("* %c\n", path[k]);
        basename[k-(sep+1)] = path[k];
        basename[k-sep] = 0;    
    }
    if (debug) pc.printf("d:<%s> b:<%s>\n", dirname, basename);
}

/** ls
 * lists files in the current working directory, 4 columns
 */
void ls(char *path) {
    if (debug) pc.printf("%s\n", cwd);
    DIR *d;
    struct dirent *p;
    
    int count=0;
    if ((d = opendir(path)) != NULL) {
        while ((p = readdir(d)) != NULL) {
            pc.printf("%12s", p->d_name);
            if (count++ >= 3) {
                count = 0;
                pc.printf("\n");
            }
        }
        pc.printf("\n");
        if (count < 3)
            pc.printf("\n");
        closedir(d);
    } else {
        pc.printf("%s: No such directory\n", path);
    }
}

/** cd
 * changes current working directory
 */
void cd(char *path) {
    strcpy(cwd, path);
}

/** pwd
 * print current working directory
 */
void pwd() {
    pc.printf("%s\n", cwd);
}

/** touch
 * create an empty file
 */
void touch(char *path) {
    FILE *fp;
    if ((fp = fopen(path, "w")) != NULL) {
        fclose(fp);
    } else {
        pc.printf("%s: No such file\n", path);
    }
} 

/** head
 * print the first 10 lines of a file
 */
void head(char *path) {
    FILE *fp;
    char line = 0;
    
    if ((fp = fopen(path, "r")) != NULL) {
        while (!feof(fp) && line++ < 10) {
            fgets(buf, 128, fp);
            pc.printf("%s", buf);
        }
        fclose(fp);
    } else {
        pc.printf("%s: No such file\n", path);
    }
}

/** cat
 * display the content of a file
 */
void cat(char *path) {
    FILE *fp;

    if ((fp = fopen(path, "r")) != NULL) {
        while (!feof(fp)) {
            if (fgets(buf, 127, fp) != NULL)
                pc.printf("%s", buf);
        }
        fclose(fp);
    } else {
        pc.printf("%s: No such file\n", path);
    }
}

/** send
 * Simple serial file transfer protocol
 * Initiates escape sequence: ^A^B, sends filename, ^C, and then file
 * contents followed by ^D
 */
void send(char *path) {
    FILE *fp;
    char dirname[32], basename[16];

    if ((fp = fopen(path, "r")) != NULL) {
        splitName(path, dirname, basename);
        pc.printf("%c%c%s%c", 1, 2, basename, 3);
        while (!feof(fp)) {
            if (fgets(buf, 127, fp) != NULL)
                pc.printf("%s", buf);
        }
        fclose(fp);
        pc.printf("%c", 4);
    } else {
        pc.printf("%s: No such file\n", path);
    }
}
