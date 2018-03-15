#include "main.h"
#include "FIFO.h"
#include "protocal.h"

/*-----USART3_TX-----PB10-----*/
/*-----USART3_RX-----PB11-----*/

FIFO_S_t* UART_TranFifo;
static unsigned char rx_buffer[256];
uint8_t receive_from_pi = 0;
void USART3_Configuration(void)
{
    USART_InitTypeDef usart3;
    GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//����GPIOBʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//����USART3ʱ��

    GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); 
	
		//GPIO_InitTypeDef  gpio;//����GPIO��ʼ���ṹ�����gpio
    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;//ѡ��Ҫ���õ�GPIO����
    gpio.GPIO_Mode = GPIO_Mode_AF;//ѡ��GPIO���ŵĹ���ģʽ
    gpio.GPIO_OType = GPIO_OType_PP;//ѡ��GPIO�����������
    gpio.GPIO_Speed = GPIO_Speed_100MHz;//ѡ��GPIO���ŵ�����
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;//ѡ��GPIO���ŵ�����/����ģʽ
    GPIO_Init(GPIOB,&gpio);//��ʼ��GPIO
	
		//USART_InitTypeDef usart3;//����USART��ʼ���ṹ�����usart3
    usart3.USART_BaudRate = 115200;//���ò�����          // speed 10byte/ms
    usart3.USART_WordLength = USART_WordLength_8b;//�����ֳ�
    usart3.USART_StopBits = USART_StopBits_1;//����ֹͣλ
    usart3.USART_Parity = USART_Parity_No;//����У��λ
    usart3.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;//USARTģʽ
    usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//Ӳ��������
    USART_Init(USART3,&usart3);//USART��ʼ��

    USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
    USART_Cmd(USART3,ENABLE);
		
		//NVIC_InitTypeDef  nvic;
    nvic.NVIC_IRQChannel = USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&nvic);

    UART_TranFifo = FIFO_S_Create(100);  
    if(!UART_TranFifo)
    {
       // while(1);  avoid while in program
	}
}


void UART3_PrintCh(uint8_t ch)
{    
    FIFO_S_Put(UART_TranFifo, ch);
    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
    
}

void UART3_PutStr (const char *str)//�����ַ���
{
    while(*str)
    {
			  USART_SendData(USART3, *str++);
			  while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){}
    }
}

void UART3_PrintBlock(uint8_t* pdata, uint8_t len)
{
	uint8_t i = 0;
    for(i = 0; i < len; i++)
    {
        FIFO_S_Put(UART_TranFifo, pdata[i]);
    }
    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);  //���ͼĴ������ж�
}


int fputc(int ch, FILE *f)
{
    while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
    USART_SendData(USART3, (uint8_t)ch);
    return ch;
}

void USART3_IRQHandler(void)
{  
    if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
    {   
        if(!FIFO_S_IsEmpty(UART_TranFifo))
        {
					uint16_t data = (uint16_t)FIFO_S_Get(UART_TranFifo);
					USART_SendData(USART3, data);
        }
        else
        {
        USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
        }  
    }else if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)    //�����ж�
    {
        uint8_t tmp = USART_ReceiveData(USART3);
			  receive_from_pi = tmp;
			  if(receive_from_pi>='1' && receive_from_pi<='9')
				{
						static int i=0;
					  static int j=0;
						switch(receive_from_pi){
							case '1':
							{
									GimbalRef.pitch_angle_dynamic_ref += 20;
									GimbalRef.yaw_angle_dynamic_ref = 5;
									CMControlLoop();
									GMPitchControlLoop();
									SetGimbalMotorOutput();
									i = 1000;
								  j = 10000;
									while(i--)
									{
										while(j--);
										j = 10000;
									}
									GimbalRef.pitch_angle_dynamic_ref -= 20;
									GimbalRef.yaw_angle_dynamic_ref = -5;
									CMControlLoop();
									GMPitchControlLoop();
									SetGimbalMotorOutput();
									i = 1000;
								  j = 10000;
									while(i--)
									{
										while(j--);
										j = 10000;
									}
							}	
							
							case '2':
							{
								GimbalRef.pitch_angle_dynamic_ref += 20;
								CMControlLoop();
								i = 10000;
								while(i--);
								GimbalRef.pitch_angle_dynamic_ref -= 20;
								CMControlLoop();
								i = 10000;
								while(i--);
								break;
							}
						  
							case '3':
							{
								GimbalRef.pitch_angle_dynamic_ref += 20;
								GimbalRef.yaw_angle_dynamic_ref = -3;
								CMControlLoop();
								i = 10000;
								while(i--);
								GimbalRef.pitch_angle_dynamic_ref -= 20;
								GimbalRef.yaw_angle_dynamic_ref = 3;
								CMControlLoop();
								i = 10000;
								while(i--);
							  break;
							}	
						
							case '4':
							{
								GimbalRef.pitch_angle_dynamic_ref += 10;
								GimbalRef.yaw_angle_dynamic_ref = 3;
								CMControlLoop();
								i = 10000;
								while(i--);
								GimbalRef.pitch_angle_dynamic_ref -= 10;
								GimbalRef.yaw_angle_dynamic_ref = -3;
								CMControlLoop();
								i = 10000;
								while(i--);
							  break;
							}	
						
							case '5':
							{
								GimbalRef.pitch_angle_dynamic_ref += 10;
								CMControlLoop();
								i = 10000;
								while(i--);
								GimbalRef.pitch_angle_dynamic_ref -= 10;
								CMControlLoop();
								i = 10000;
								while(i--);
							  break;
							}
						
							case '6':
							{
								GimbalRef.pitch_angle_dynamic_ref += 10;
								GimbalRef.yaw_angle_dynamic_ref = -3;
								CMControlLoop();
								i = 10000;
								while(i--);
								GimbalRef.pitch_angle_dynamic_ref -= 10;
								GimbalRef.yaw_angle_dynamic_ref = 3;
								CMControlLoop();
								i = 10000;
								while(i--);
							  break;
							}	
						
							case '7':
							{
								GimbalRef.yaw_angle_dynamic_ref = 3;
								CMControlLoop();
								i = 10000;
								while(i--);
								GimbalRef.yaw_angle_dynamic_ref = -3;
								CMControlLoop();
								i = 10000;
								while(i--);
							  break;
							}	
						
							case '8':
							{
								break;
							}
						
							case '9':
							{
								GimbalRef.yaw_angle_dynamic_ref = -3;
								CMControlLoop();
								i = 10000;
								while(i--);
								GimbalRef.yaw_angle_dynamic_ref = 3;
								CMControlLoop();
								i = 10000;
								while(i--);
							  break;
							}
						
							default:
							{
							
							}
					}
					receive_from_pi = 0;
				}
        if(FrameUnpack(tmp, rx_buffer))
        {
            if(GetFrameCmd(rx_buffer)  == CMD_ID_SENSOR_CALI)     //������У׼����ID
            {
                CALI_CMD *pData = (CALI_CMD *)GetFrameDataAddress(rx_buffer);	
                //get the pData->type									
                if(pData->type == GYRO_CALI_START)
                {
                    SetCaliCmdFlag(CALI_START_FLAG_GYRO);
                }
                else if(pData->type == GYRO_CALI_END)
                {
                    SetCaliCmdFlag(CALI_END_FLAG_GYRO);
                }	
                else if(pData->type == MAG_CALI_START)
                {
                    SetCaliCmdFlag(CALI_START_FLAG_MAG);
                }
                else if(pData->type == MAG_CALI_END)
                {
                    SetCaliCmdFlag(CALI_END_FLAG_MAG);
                }
                else if(pData->type == Encoder_CALI_START)
                {
                    SetCaliCmdFlag(CALI_START_FLAG_GIMBAL);
                }	
                else if(pData->type == Encoder_CALI_END)
                {
                    SetCaliCmdFlag(CALI_END_FLAG_GIMBAL);
                }
            }
            else if(GetFrameCmd(rx_buffer)  == CMD_ID_UPLOAD_CALI_INFO)  //�����Ҫ���ϴ�У׼��Ϣ
            {
            }
            else if(GetFrameCmd(rx_buffer) == CMD_ID_PID_CALI)				//����PID����У׼����
            {
                PIDParamStruct_t *pData = (PIDParamStruct_t *)GetFrameDataAddress(rx_buffer);
                PIDCaliProcess(pData);   //����calidata��static������
                SetCaliCmdFlag(CALI_FLAG_PID);	//PID����							
            }
            else
            {

            }
        }
    }       
}

