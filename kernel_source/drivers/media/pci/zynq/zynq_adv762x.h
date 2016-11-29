#ifndef _ZYNQ_ADV762X_H_
#define _ZYNQ_ADV762X_H_



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
    EDIDACTIVE_AND_CABLEDECT = 2,
    EDIDACTIVE_AND_CABLEDECT_ADN_MAN = 3,
    MAN = 4
} ADV762X_HPA;



typedef enum {
    FS128= 0, //1x128
    FS256 = 1, //2x128
    FS384 =  2, //3x128
    FS512 = 3, //4x128
    FS640 = 4, //5x128
    FS768 =5 //6x128
} ADV762X_MCLK_DIV;


struct adv762x_platform_data {
    ADV762x_HDMI_SRC_ID hdmi_src_id;
    ADV762x_MODE mode;
    unsigned int rx_main_map;
    unsigned int edid_config_map;
    unsigned int rx_repeater_map;
    unsigned int rx_information_map;
    unsigned int rx_edid_map;
    unsigned int rx_test_map;
    unsigned int cp_lite_map;
    unsigned int dpll_map;
    unsigned int osd_map;
    unsigned int tx_main_map;
    unsigned int tx_packet_map;
    unsigned int tx_cec_map;
    unsigned int tx_edid_map;
    unsigned int tx_test_map;
    unsigned int is_initial_interrupt;
    unsigned int adv762x_use_fixed_edid;
};


#endif	/*_ZYNQ_PCM1865_H_ */