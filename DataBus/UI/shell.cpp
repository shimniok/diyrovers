#include "mbed.h"
#include "DirHandle.h"
#include "SDHCFileSystem.h"
#include "util.h"
#include "Buttons.h"

#define MAXCMDARR	18
#define MAXCMD		16
#define MAXDESC		32

extern Serial pc;
extern Buttons keypad;

char cwd[64];
char buf[128];
int status;
bool debug=false;
bool done=false;

typedef struct {
	const char *cmd;
	int (*f)(const char *arg0);
	const char *desc;
} cmd;

void shell(void);
//void addcmd(const char *cmd, int (*f)(const char *arg0), const char *desc);
void docmd(char *cmdline);
void termInput(char *cmd);
void resolveDirectory(char *newpath, char *path);
void splitName(const char *path, char *dirname, char *basename);
int dols(const char *path);
int docd(const char *path);
int dopwd(const char *s);
int dotouch(const char *path);
int domkdir(const char *path);
int dohead(const char *path);
int docat(const char *path);
int dosend(const char *path);
int doexit(const char *s);
int dodebug(const char *s);
int dohelp(const char *s);
int doinstrchk(const char *s);
int docompswing(const char *s);
int dogyroswing(const char *s);
int doreset(const char *s);
int doautonomous(const char *s);

cmd command[MAXCMDARR] = {
		{ "ls", dols, "list files" },
    	{ "cd", docd, "change directory" },
    	{ "pwd", dopwd, "print working directory" },
    	{ "touch", dotouch, "create, update file" },
    	{ "mkdir", domkdir, "make directory" },
    	{ "head", dohead, "output first part of file" },
    	{ "cat", docat, "output file" },
    	{ "send", dosend, "send file to terminal" },
    	{ "rm", remove, "remove file" },
    	{ "debug", dodebug, "toggle debug mode" },
    	{ "exit", doexit, "exit shell" },
    	{ "help", dohelp, "print this help" },
    	{ "stat", doinstrchk, "instrument check" },
    	{ "comp", docompswing, "compass swing" },
    	{ "gyro", dogyroswing, "gyro swing" },
    	{ "reset", doreset, "reset the MCU" },
    	{ "auto", doautonomous, "run autonomous mode" },
    	{ 0, 0, 0 }
};


void shell() {
    FILE *fp;
    char cmdline[64];
    /*
    if ((fp = fopen("/log/message.txt", "w")) != NULL) {
        for (int i=0; i < 20; i++)
            fprintf(fp, "Hello, World!\n");
        fclose(fp);
    } else {
        pc.printf("Error creating file\n");
    }
    pc.printf("\n");
    */

    pc.printf("Type help for assistance\n");
    
    strcpy(cwd, "/log");

    status=0;
    done=false;
    while (!done) {
        termInput(cmdline);

        // interrupt operation if keypad button is pressed
        if (keypad.pressed) {
            keypad.pressed = false;
            break;
        }        

        docmd(cmdline);
        
    }
    pc.printf ("exiting shell\n");

    return;
}


/** docmd
 * Run a command by looking it up the command requested on the shell command line in the
 * command array.  If it's found, run the associated function. If not, print an error.
 */
void docmd(char *cmdline) {
	char *arg;
	char cmd[64];
	char newpath[64];
	bool found = false;

    arg = split(cmd, cmdline, 64, ' ');

    if (strlen(cmd) > 0) {

		resolveDirectory(newpath, arg);

		if (debug) pc.printf("cmdline:<%s> cmd:<%s> arg:<%s> newpath:<%s>\n", cmdline, cmd, arg, newpath);

		for (int i=0; command[i].cmd; i++) {
			if (!strcmp(cmd, command[i].cmd)) {
				found = true;
				command[i].f(newpath);
			}
		}

		if (!found) {
			pc.printf("%s: command not found\n", cmd);
		}

    }

	return;
}


/** termInput
 * read input from the terminal
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

/** splitCmd
 * copy t to s until space is reached
 * return location of delimiter+1 in t
 * if space not found, return t pointing to end of string
 * if s or t null, return null
 */
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
void splitName(const char *path, char *dirname, char *basename) {
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
    for (unsigned int k=sep+1; k != strlen(path); k++) {
        if (debug) pc.printf("* %c\n", path[k]);
        basename[k-(sep+1)] = path[k];
        basename[k-sep] = 0;    
    }
    if (debug) pc.printf("d:<%s> b:<%s>\n", dirname, basename);
}

/** ls
 * lists files in the current working directory, 4 columns
 */
int dols(const char *path) {
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
        status = 0;
    } else {
        pc.printf("%s: No such directory\n", path);
        status = 1;
    }

    return status;
}

/** cd
 * changes current working directory
 */
int docd(const char *path) {
    strcpy(cwd, path);

    return 0;
}

/** pwd
 * print current working directory
 */
int dopwd(const char *s) {
    pc.printf("%s\n", cwd);

    return 0;
}

/** touch
 * create an empty file
 */
int dotouch(const char *path) {
    FILE *fp;
    if ((fp = fopen(path, "w")) != NULL) {
        fclose(fp);
        status = 0;
    } else {
        pc.printf("%s: No such file\n", path);
        status = 1;
    }

    return status;
} 

int domkdir(const char *path) {
	return mkdir(path, S_IRWXU|S_IRWXG|S_IRWXO);
}

/** head
 * print the first 10 lines of a file
 */
int dohead(const char *path) {
    FILE *fp;
    char line = 0;
    
    if ((fp = fopen(path, "r")) != NULL) {
        while (!feof(fp) && line++ < 10) {
            fgets(buf, 128, fp);
            pc.printf("%s", buf);
        }
        fclose(fp);
        status = 0;
    } else {
        pc.printf("%s: No such file\n", path);
        status = 1;
    }

    return status;
}

/** cat
 * display the content of a file
 */
int docat(const char *path) {
    FILE *fp;

    if ((fp = fopen(path, "r")) != NULL) {
        while (!feof(fp)) {
            if (fgets(buf, 127, fp) != NULL)
                pc.printf("%s", buf);
        }
        fclose(fp);
        status = 0;
    } else {
        pc.printf("%s: No such file\n", path);
        status = 1;
    }

    return status;
}

/** send
 * Simple serial file transfer protocol
 * Initiates escape sequence: ^A^B, sends filename, ^C, and then file
 * contents followed by ^D
 */
int dosend(const char *path) {
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
        status = 0;
    } else {
        pc.printf("%s: No such file\n", path);
        status = 1;
    }

    return status;
}

/** doexit
 * set a flag to exit the shell
 */
int doexit(const char *s) {
	done = true;

	return 0;
}

/** dodebug
 * toggle the debug state variable
 */
int dodebug(const char *s) {
	debug = !debug;

	return 0;
}

/** dohelp
 * print the list of commands and descriptions
 */
int dohelp(const char *s) {
	for (int i=0; command[i].cmd; i++) {
		pc.printf("%10s %s\n", command[i].cmd, command[i].desc);
	}

	return 0;
}

/** doinstrchk
 * call external instrument check routine
 */
int doinstrchk(const char *s) {
	extern void displayData(int mode);
	displayData(0);

	return 0;
}

/** docompswing
 * perform compass swing, call external function
 */
int docompswing(const char *s) {
	extern int compassSwing();
	compassSwing();

	return 0;
}

/** dogyroswing
 * perform gyro swing, call external function
 */
int dogyroswing(const char *s) {
	extern int gyroSwing();
	gyroSwing();

	return 0;
}

/** doreset
 * reset the processor
 */
int doreset(const char *s) {
	extern int resetMe();
	resetMe();
	return 0; // won't ever reach this line...
}

/** doautonomous
 * call external doAutonomous mode to perform an autonomous run
 */
int doautonomous(const char *s) {
	extern int autonomousMode();
	autonomousMode();

	return 0;
}
