#include<stdio.h>
#include<stdlib.h>
 #include <unistd.h>
int main()
{
int a,b,id;
system("stty -F /dev/ttyUSB0 115200 -parity -cstopb");
FILE *p;
char s[100];
while(1)
{
p=fopen("/dev/ttyUSB0","r");
fseek(p, -7, SEEK_END);
fscanf(p,"$%d *%d #%d",&id,&a,&b);

if(a>=0 && a<=999 && b>=0 && b<=999)
{
printf("%d--%d\n",a,b);
sprintf(s,"wget \"http://livemonitoring.info/wsn/ins.php?nid=%d&temp=%d&light=%d\"",id,a,b); 
//http://livemonitoring.info/wsn/ins.php?nid=1&temp=30&light=100
printf("%s",s);
system(s);
}
sleep(5);
fclose(p);
}

return 0;
}
