#include<stdint.h>
#include<stdio.h>
uint8_t Cong(uint8_t a,uint8_t b){
	printf("Cong = %d\n",a+b);
}
uint8_t Tru(uint8_t a,uint8_t b){
	printf("tru c2 = %d\n",a-b);
}
uint8_t Nhan(uint8_t a,uint8_t b){
	printf("nhan c3 = %d\n",a*b);
}
float Chia(float a,float b){
	printf("Chia c4 = %f",(float)a/b);	
}
int main(){
	uint8_t var1=5;
	uint8_t var2=3;
	Cong(var1,var2);
	Tru(var1,var2);
	Nhan(var1,var2);
	Chia(var1,var2);
	return 0;
}
