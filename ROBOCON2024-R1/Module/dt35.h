#ifndef DT35_H
#define DT35_H

#define data_lenth   28
#define dt35_huart  huart1

typedef struct 
{
	float current;
	float distance;
	float current2distance;
}dt35_t;

void usart_init(void);
void dt35_send(void);
float dt35_read(dt35_t *dt35);
float dt35_data_parsing(void);


#endif // !


