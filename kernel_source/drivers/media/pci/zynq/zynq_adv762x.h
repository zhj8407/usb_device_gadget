#ifndef _ZYNQ_ADV762X_H_
#define _ZYNQ_ADV762X_H_

typedef enum {
	ADV7625 = 0,
	ADV7627 = 1,
	ADV762X = 2
} ADV762X_ID;

typedef enum {
	TK1HDMI = 0, //RXC
	CON18HDMI = 1, //RXE
	UNKNOWNHDMI = 2 //RXB
} ADV762x_HDMI_SRC_ID;


typedef enum {
	MUXMODE = 0, //Mux Mode
	TRANSMODE = 1, //Transceiver Mode
	UNKNOWNMODE = 2 //UNKNOWNMODE
} ADV762x_MODE;

//hot plug assert (HPA) controls
typedef enum {
	EDIDACTIVE = 0,
	CABLEDECT =1,
	EDIDACTIVE_AND_CABLEDECT = 2
} ADV762X_HPA;



typedef enum {
	FS128= 0, //1x128
	FS256 = 1, //2x128
	FS384 =  2, //3x128
	FS512 = 3, //4x128
	FS640 = 4, //5x128
	FS768 =5 //6x128
} ADV762X_MCLK_DIV;


// [RXA, RxB, RXC, RXD, RXE]---->rx1 ---> txA
// [RXA, RxB, RXC, RXD, RXE]---->rx2 ---> txB
struct adv762x_pin_config {
	unsigned int rx1_enable;
	unsigned int rx2_enable;
	unsigned int rx1_source; //0:RXA, 1:RXB, 2:RXC, 3:RXD, 4:RXE
	unsigned int rx2_source;//0:RXA, 1:RXB, 2:RXC, 3:RXD, 4:RXE
	unsigned int txA_enable;
	unsigned int txB_enable;
};

struct adv762x_platform_data {
    ADV762X_ID id;
	ADV762x_HDMI_SRC_ID hdmi_src_id;
	ADV762x_MODE mode;
	struct adv762x_pin_config pin_config;
	unsigned int rx_main_map[2];
	unsigned int rx_repeater_map[2];
	unsigned int rx_information_map[2];
	unsigned int rx_edid_map[2];
	unsigned int rx_test_map;
	unsigned int cp_lite_map[2];
	unsigned int dpll_map[2];
	unsigned int osd_map;
	unsigned int tx_main_map[2];
	unsigned int tx_packet_map[2];
	unsigned int tx_cec_map[2];
	unsigned int tx_edid_map[2];
	unsigned int tx_test_map[2];
};


#endif	/*_ZYNQ_PCM1865_H_ */