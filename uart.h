#define BAUD_PRESCALE 103
#define CLK_SPEED 16000000
#define BAUD 9600

void usart_init(void);

void usart_send( uint8_t data );

uint8_t usart_recv(void);

uint8_t usart_istheredata(void);

void usart_display_float(float bigval);
