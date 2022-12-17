
#ifndef CONSOLE_H    // если имя CPPSTUDIO_H ещё не определено
#define CONSOLE_H 


#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <string.h> 
#include <ctype.h>
#include <math.h>


#define LEN_TEMPLATE_IN  6
#define LEN_TEMPLATE_GET  14

/*
 * Вариант 4
 * Функция ln(x)
 *
*/

void Message_Write_2_Buffer(char buffer[], float result)
{
	sprintf(buffer, "\nln(x)=%06.3f\r", trunc(result * 1000.0) / 1000.0);
}

void Clear_Console()
{
    char c;
    while ( (c = getchar()) != '\n' && c != EOF ) { }
}

int Message_Check(char buf[])
{
    unsigned char template[] = "Nx=**E";
    if (memcmp(buf, template, 3))
        return 0;
    
    if (buf[3] > '9' || buf[3] < '0' 
        || buf[4] > '9' || buf[4] < '0')
        return 0;
    
    if (buf[5] != 'E')
        return 0;
    
    return 1;
}

int Message_Get_Num(char buf[])
{
    return atoi(buf+3);
}

#ifdef __linux__ 

int Message_Read_From_Console(char buf[])
{
    uint32_t tryCount = 0;
    for(;;){
        
        if (tryCount >= 10) return -1;
        
        size_t count = read(STDIN_FILENO, buf, LEN_TEMPLATE_IN);
        clearConsole();
        if (count < LEN_TEMPLATE_IN){
            printf("To small message. Please retry.\n");
            tryCount++;
        } else {
            if (checkMessage(buf))
                break;
            else{
                printf("Wrong format. Please retry.\n");
                tryCount++;
            }
        }
    }
    return 0;
}

#elif _WIN32
int Message_Read_From_Console(char buf[])
{
    uint32_t tryCount = 0;
    for(;;){
        
        if (tryCount >= 10) return -1;
        
        printf("\nWrite message: ");
        scanf("%s", buf);
        size_t count = strlen(buf);
        
        clearConsole();
        if (count < LEN_TEMPLATE_IN){
            printf("To small message. Please retry.\n");
            tryCount++;
        } else {
            if (checkMessage(buf))
                break;
            else{
                printf("Wrong format. Please retry.\n");
                tryCount++;
            }
        }
    }
    return 0;
}
#else

#endif


// полезно про формат и как задается
// https://en.cppreference.com/w/c/io/fscanf
// https://stackoverflow.com/questions/20912166/how-to-test-that-a-string-follows-a-given-pattern-in-c


#endif CONSOLE_H