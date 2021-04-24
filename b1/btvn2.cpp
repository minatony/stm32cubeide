#include<stdio.h>
#include<stdint.h>
int main(){
	uint16_t count=0;
	for(uint16_t i=0;i<=350;i++){
		printf("count[%d]=%d\n",i,count++);
	}
	return 0;
}
