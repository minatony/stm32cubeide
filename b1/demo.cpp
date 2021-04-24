/*#include<stdio.h>
#include<stdint.h>
int main(){
	char a[]="tapitstm32k10";
	for(int i=0;i<13;i++){
		printf("%c\n",a[i]);
	}
	return 0;
}*/
#include <stdio.h>
void function(){  
int x = 20;//local variable  
static int y = 30;//static variable  
x = x + 10;  
y = y + 10;  
printf("\n%d,%d",x,y);  
}  
int main() {
 
  function();
  function();
  function();
  return 0;
}
