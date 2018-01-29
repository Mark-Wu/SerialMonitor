
#include<stdio.h>      /*标准输入输出定义*/
#include <time.h>
#include <sys/time.h>
#include<stdlib.h>     /*标准函数库定义*/
#include<unistd.h>     /*Unix 标准函数定义*/
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>      /*文件控制定义*/
#include<termios.h>    /*PPSIX 终端控制定义*/
#include<errno.h>      /*错误号定义*/
#include<string.h>
#include <stdbool.h>
#include <signal.h>
#include "jRead.h"
#include "queue_list.h"

#define  __THREAD

#ifdef __THREAD
#include <pthread.h>
#endif

#define     MAX_SIZE    1024
#define FILE_BUFFER_MAXLEN 1024*1024

bool __init = true;
char pserial_dir[128] = {0};

char plog_path[128] = {0};

int band_rate = 115200;
int flow_ctl = 0;
int data_bits = 8;
int stop_bits = 1;
char parity[32] = {0};
int rec_delay = 100;
int log_fd;
Queue ptr_queue;
pthread_mutex_t queue_lock;


struct FileBuffer{
    unsigned long length;			// length in bytes
    unsigned char *data;			// malloc'd data, free with freeFileBuffer()
};


static void myquit(int signum)
{
    __init = false;
}

static void setsignal(void)
{
    struct sigaction act;
    memset(&act, 0, sizeof(act));
    act.sa_handler = myquit;
    sigaction(SIGTERM, &act, NULL);
    sigaction(SIGHUP, &act, NULL);
    sigaction(SIGINT, &act, NULL);
}

int serial_recv(int fd, char *rcv_buf,int data_len)
{
    int len,temp,fs_sel;
    fd_set fs_read;
    char *pos = rcv_buf;

    struct timeval time;

    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);
    time.tv_sec = 5;
    time.tv_usec = 0;
    len = 0;
    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);
    if(fs_sel) {
#if 0
        for(pos = rcv_buf;len < data_len; pos += temp ){
            temp = read(fd,pos,data_len - len);
            if(temp <= 0)
                break;
            else{
                len += temp;
            }
         }
#else
        usleep(1000*rec_delay);
        len = read(fd,rcv_buf,data_len);
#endif

        return len;
    } else{
        return -1;
    }

}
int serial_set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{

    int   i;
    int   status;
    int   speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};
    int   name_arr[] = {115200,  19200,  9600,  4800,  2400,  1200,  300};

    struct termios options;

    /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.
    */
    if  ( tcgetattr( fd,&options)  !=  0)
    {
        perror("SetupSerial 1");
        return -1;
    }

    //设置串口输入波特率和输出波特率
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
    {
        if  (speed == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }

    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    //设置数据流控制
    switch(flow_ctrl)
    {

        case 0 ://不使用流控制
            options.c_cflag &= ~CRTSCTS;
            break;

        case 1 ://使用硬件流控制
            options.c_cflag |= CRTSCTS;
            break;
        case 2 ://使用软件流控制
            options.c_cflag |= IXON | IXOFF | IXANY;
            break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
        case 5    :
            options.c_cflag |= CS5;
            break;
        case 6    :
            options.c_cflag |= CS6;
            break;
        case 7    :
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            fprintf(stderr,"Unsupported data size\n");
            return -1;
    }
    //设置校验位
    switch (parity)
    {
        case 'n':
        case 'N': //无奇偶校验位。
            options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            break;
        case 'o':
        case 'O'://设置为奇校验
            options.c_cflag |= (PARODD | PARENB);
            options.c_iflag |= INPCK;
            break;
        case 'e':
        case 'E'://设置为偶校验
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK;
            break;
        case 's':
        case 'S': //设置为空格
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            fprintf(stderr,"Unsupported parity\n");
            return -1;
    }
    // 设置停止位
    switch (stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB; break;
        case 2:
            options.c_cflag |= CSTOPB; break;
        default:
            fprintf(stderr,"Unsupported stop bits\n");
            return -1;
    }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//我加的
//options.c_lflag &= ~(ISIG | ICANON);

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */

    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd,TCIFLUSH);

    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("com set error!\n");
        return -1;
    }
    return 0;
}

int serial_init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    //设置串口数据帧格式
    if (serial_set(fd,speed,flow_ctrl,databits,stopbits,parity) == -1)
        return -1;
    else
        return  0;
}
#ifdef __THREAD

static void * write_log(void *arg)
{
    int log_fd;
    uint32_t pacakge_address;
    char *package = NULL;
    printf("sub thread log path: %s .\r\n",plog_path);

    log_fd = open(plog_path,O_RDWR | O_APPEND | O_CREAT);
    if(log_fd == -1) {
        printf("error is %s\n", strerror(errno));
        exit(1);
    }
    while (__init){
        while(IsEmpty(&ptr_queue) == 1){
            usleep(1000);
        }
        pthread_mutex_lock(&queue_lock);
        DeQueue(&ptr_queue,&pacakge_address);
        pthread_mutex_unlock(&queue_lock);
        package = (char *)pacakge_address;
        write(log_fd,package,strlen(package));
        free(package);
    }
    printf("log thread is end.");
    close(log_fd);
    return NULL;
}

#endif
static unsigned int fast_atoi( char *p )
{
    unsigned int x = 0;
    while (*p >= '0' && *p <= '9') {
        x = (x*10) + (*p - '0');
        ++p;
    }
    return x;
};
static void print_tips(void)
{

    printf("SerialMonitor -c <config file path >\r\n");
    printf("    eg: SerialMonitor -c /etc/serialmonitor.json \r\n\r\n");
    printf("config file format example:\r\n");
    printf("{\n"
                   "        \"serial path\":\"/dev/ttyUSB0\",\n"
                   "        \"log path\":\"/var/run/serial.log\",\n"
                   "        \"band rate\":9600,\n"
                   "        \"flow control\":0,\n"
                   "        \"data bits\":8,\n"
                   "        \"stop bits\":1,\n"
                   "        \"parity\":\"N\"\n"
                   "}\r\n");
    return;
}


void freeFileBuffer( struct FileBuffer *buf )
{
    if( buf->data != NULL )
        free( buf->data );
    buf->data= 0;
    buf->length= 0;
}

unsigned long readFileBuffer( char *filename, struct FileBuffer *pbuf, unsigned long maxlen )
{
    FILE *fp;
    int i;

    if( (fp=fopen(filename, "rb")) == NULL )
    {
        printf("Can't open file: %s\n", filename );
        return 0;
    }
    // find file size and allocate buffer for JSON file
    fseek(fp, 0L, SEEK_END);
    pbuf->length = ftell(fp);
    if( pbuf->length >= maxlen )
    {
        fclose(fp);
        return 0;
    }
    // rewind and read file
    fseek(fp, 0L, SEEK_SET);
    pbuf->data= (unsigned char *)malloc( pbuf->length + 1 );
    memset( pbuf->data, 0, pbuf->length+1 );	// +1 guarantees trailing \0

    i= fread( pbuf->data, pbuf->length, 1, fp );
    fclose( fp );
    if( i != 1 )
    {
        freeFileBuffer( pbuf );
        return 0;
    }
    return pbuf->length;
}

int json_configs_read(char *config_path)
{
    int ret = 0;
    struct FileBuffer json;
    struct jReadElement jElement;

    if( readFileBuffer( config_path, &json, FILE_BUFFER_MAXLEN ) == 0 )
    {
        printf("Can't open file: %s\n", config_path );
        ret = -1;
        return ret;
    }
    // perform query on JSON file
    jRead( (char *)json.data, "{'serial path'" , &jElement );
    memcpy(pserial_dir,jElement.pValue,(size_t)jElement.bytelen);
    printf( " serial path = %s \r\n",pserial_dir);

    jRead( (char *)json.data, "{'log path'" , &jElement );
    memcpy(plog_path,jElement.pValue,(size_t)jElement.bytelen);
    printf( " log path = %s \r\n",plog_path);

    jRead( (char *)json.data, "{'band rate'" , &jElement );
    band_rate = fast_atoi(jElement.pValue);
    printf( " band rate = %d \r\n",band_rate);
    jRead( (char *)json.data, "{'flow control'" , &jElement );
    flow_ctl = fast_atoi(jElement.pValue);
    printf( " flow control = %d \r\n",flow_ctl);
    jRead( (char *)json.data, "{'data bits'" , &jElement );
    data_bits = fast_atoi(jElement.pValue);
    printf( " data bits = %d \r\n",data_bits);
    jRead( (char *)json.data, "{'stop bits'" , &jElement );
    stop_bits = fast_atoi(jElement.pValue);
    printf( " stop bits = %d \r\n",stop_bits);
    jRead( (char *)json.data, "{'parity'" , &jElement );
    memcpy(parity,jElement.pValue,1);
    printf(" parity = %c\r\n",parity[0]);
    jRead( (char *)json.data, "{'rec delay'" , &jElement );
    rec_delay = fast_atoi(jElement.pValue);
    printf( " rec delay = %d ms\r\n",rec_delay);

    freeFileBuffer( &json );

    return  ret;

}
int main(int argc,char **argv)
{
    int serial_fd;

    int err;
    int len;
    char rcv_buf[MAX_SIZE];
    char *pstr_buffer;
    char *pstr = NULL;
    struct timeval ts;
    int i;
#ifdef __THREAD
    pthread_t id;
    pthread_attr_t attr;
#endif

    if(argc != 3){
        print_tips();
        exit(0);
    }

    for ( i = 1; i < argc;i++) {
        char *arg=argv[i];
        if (strcmp(arg,"-c")==0 || strcmp(arg,"--config_file ") == 0) {	    /*  serial configs file path  */
            printf("%s\r\n",argv[i]);
            if (++i<argc) {

                err = json_configs_read(argv[i]);
                if(err == -1){
                    printf("error : json.\r\n");
                    exit(1);
                }
            } else {
                printf("error: config file path expected after %s option.\n",arg);
                exit(1);
            }
        }else{
            print_tips();
            exit(0);
        }

    }
   setsignal();

#ifdef __THREAD

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    pthread_mutex_init(&queue_lock,NULL);
    pthread_create(&id,&attr,write_log,NULL);

#endif
    serial_fd = open(pserial_dir, O_RDWR | O_NONBLOCK);
    if(serial_fd == -1){
        printf("\r\n open %s error.\r\n",pserial_dir);
        exit(1);
    }
    else
        printf("\r\n open %s successfully.\r\n",pserial_dir);

    /* set serial to block status */
    if(fcntl(serial_fd, F_SETFL, 0) < 0)
    {
        printf("fcntl failed!\n");
        close(serial_fd);
        return -1;
    }
    else
        printf("fcntl=%d\n",fcntl(serial_fd, F_SETFL,0));

    do{
        err = serial_init(serial_fd,band_rate,flow_ctl,data_bits,stop_bits,parity[0]);
        printf("Set Port Exactly!\n");
    }while(-1 == err || -1 == serial_fd);


    while (__init)
    {
        len = serial_recv(serial_fd, rcv_buf,MAX_SIZE);
        printf("rec len = %d\r\n",len);
        if(len > 0)
        {
            pstr_buffer = (char *)malloc(MAX_SIZE * 4);
            memset(pstr_buffer,0x00,MAX_SIZE * 4);
            gettimeofday(&ts,NULL);
            pstr = pstr_buffer;
            pstr += sprintf(pstr,"%ld(s)+%ld(us) : ",ts.tv_sec,ts.tv_usec );
            for (i = 0; i < len; ++i) {
                pstr += sprintf(pstr,"%02X ",(unsigned char)rcv_buf[i]);
            }
            pstr += sprintf(pstr,"\r\n");
            printf("%s",pstr_buffer); 
            pthread_mutex_lock(&queue_lock);
            EnQueue(&ptr_queue,(uint32_t)pstr_buffer);
            pthread_mutex_unlock(&queue_lock);
        }
        usleep(1000);
    }

    close(serial_fd);

    printf(" serial port monitor closed.\r\n");
    return 0;
}
