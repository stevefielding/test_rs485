#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <wiringPi.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

// enable loopback by keeping the rs485_en active during transmission
#define RS485_LOOPBACK

// Uses wiringpi numbering. 
// from terminal "gpio readall" to get mapping to BCM numbers
const uint8_t RS485_TX_EN = 4;
const uint8_t RS485_RX_EN_N = 5;
const uint8_t RS232_RX_EN_N = 0;

// Init the rs485 enables and in this case an rs232 enable.
// Designed for hardware with rs232/rs485 mux and seperate rs485 
// rx and tx enables. Seperate enables are handy for performing rs485 loopback
// Your hardware probably won't have rs232_en.
int init() {
  wiringPiSetup();
  pinMode(RS485_TX_EN, OUTPUT);
  pinMode(RS485_RX_EN_N, OUTPUT);
  pinMode(RS232_RX_EN_N, OUTPUT);
  digitalWrite(RS485_TX_EN, LOW);
  digitalWrite(RS485_RX_EN_N, LOW);
}

ssize_t writec(int fd, char *buf, size_t count) {
  digitalWrite(RS485_TX_EN, HIGH);
#ifndef RS485_LOOPBACK
  digitalWrite(RS485_RX_EN_N, HIGH);
#endif
  ssize_t r = write(fd, buf, count);
  uint8_t lsr;
  do {
    int r = ioctl(fd, TIOCSERGETLSR, &lsr);
  } while (!(lsr & TIOCSER_TEMT));
  digitalWrite(RS485_TX_EN, LOW);
#ifndef RS485_LOOPBACK
  digitalWrite(RS485_RX_EN_N, LOW);
#endif
  return r;
}

void main() {
int fd1;
int rd;
unsigned char buff[256];

  init();
  fd1=open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
  if (fd1 == -1 ) {
    perror("open_port: Unable to open /dev/ttyAMA0 – ");
  }
  else {
    fcntl(fd1, F_SETFL,0);
    printf("Port 1 has been sucessfully opened and %d is the file description\n",fd1);
  }
  //CONFIGURE THE UART
  //The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
  //	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
  //	CSIZE:- CS5, CS6, CS7, CS8
  //	CLOCAL - Ignore modem status lines
  //	CREAD - Enable receiver
  //	IGNPAR = Ignore characters with parity errors
  //	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
  //	PARENB - Parity enable
  //	PARODD - Odd parity (else even)
  struct termios options;
  tcgetattr(fd1, &options);
  options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcflush(fd1, TCIFLUSH);
  tcsetattr(fd1, TCSANOW, &options);

  while (1) {
    printf("Sending: \"5555\"\n");
    writec(fd1, "5555", 4);
    rd = read(fd1, (void *) buff, 4);
    if (rd > 0) {
      buff[rd] = '\0';
      printf("Received: \"%s\"\n", buff);
    }
    else {
      printf("[ERROR] Receive return code: %d\n", rd);
    }
    sleep(2);
  }
}

