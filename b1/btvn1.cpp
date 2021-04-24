#include<stdio.h>
#include<string.h>
int main(){
	//char str[13]={'T','a','p','i','t','s','t','m','3','2','k','1','0'};
	char str[]="Tapitstm32k10";
	printf("in ra tung ki tu trong chuoi theo tung hang\n");
	for(int i=0;i<strlen(str);i++){
		printf("str[%d] = %c\n",i,str[i]);
	}
	return 0;
}
