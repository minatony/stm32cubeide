#include <string.h>
#include <stdio.h>
#include<stdint.h>
int main()
{
   char str[80] = "Hoc C - co ban va nang cao - tai VietJack";
   const char s[2] = "-";
   char *token;
   int a[5]= {1,2,3,4};
   printf("%d",a[0]);
   /* lay token dau tien */
/*   token = strtok(str, s);
   printf( " %s\n", token ); */
   /* duyet qua cac token con lai */
/*   while( token != NULL ) 
   {
      printf( " %s\n", token );
    
      token = strtok(NULL, s);
   }*/
   
   return(0);
}
