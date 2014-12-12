


#ifndef __CUST_DISPLAY_H__
#define __CUST_DISPLAY_H__

// color
#define BAR_OCCUPIED_COLOR  (0x07E0)    // Green
#define BAR_EMPTY_COLOR     (0xFFFF)    // White
#define BAR_BG_COLOR        (0x0000)    // Black




#if defined(FHD) || defined(CU_FHD) || defined(CMCC_FHD)
	// fhd 1080*1920
	
	// battery capacity rectangle
	#define CAPACITY_LEFT                (387) // battery capacity center
	#define CAPACITY_TOP                 (802)
	#define CAPACITY_RIGHT               (691)
	#define CAPACITY_BOTTOM              (1292)

	// first number rectangle
	#define NUMBER_LEFT                  (351+84) // number
	#define NUMBER_TOP                   (479)
	#define NUMBER_RIGHT                 (435+84)
	#define NUMBER_BOTTOM                (600)

	// %  rectangle
	#define PERCENT_LEFT                 (519+84) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (479)
	#define PERCENT_RIGHT                (627+84)
	#define PERCENT_BOTTOM               (600)

	// top animation part
	#define TOP_ANIMATION_LEFT           (387) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (691)
	#define TOP_ANIMATION_BOTTOM         (152)

	// for old animation
	#define BAR_LEFT            (470)
	#define BAR_TOP             (356)
	#define BAR_RIGHT           (610)
	#define BAR_BOTTOM          (678)

#elif defined(HD720) || defined(CU_HD720) || defined(CMCC_HD720)
	// hd720 720*1280

	// battery capacity rectangle
	#define CAPACITY_LEFT                (278) // battery capacity center
	#define CAPACITY_TOP                 (556)
	#define CAPACITY_RIGHT               (441)
	#define CAPACITY_BOTTOM              (817)

	// first number rectangle
	#define NUMBER_LEFT                  (290) // number
	#define NUMBER_TOP                   (386)
	#define NUMBER_RIGHT                 (335)
	#define NUMBER_BOTTOM                (450)

	// %  rectangle
	#define PERCENT_LEFT                 (380) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (386)
	#define PERCENT_RIGHT                (437)
	#define PERCENT_BOTTOM               (450)

	// top animation part
	#define TOP_ANIMATION_LEFT           (278) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (441)
	#define TOP_ANIMATION_BOTTOM         (129)

	// for old animation
	#define BAR_LEFT            (313)
	#define BAR_TOP             (238)
	#define BAR_RIGHT           (406)
	#define BAR_BOTTOM          (453)

#elif defined(FWVGA) || defined(CU_FWVGA) || defined(CMCC_FWVGA)
	// fwvga 480*854

	// battery capacity rectangle
	#define CAPACITY_LEFT                (172) // battery capacity center
	#define CAPACITY_TOP                 (357)
	#define CAPACITY_RIGHT               (307)
	#define CAPACITY_BOTTOM              (573)

	// first number rectangle
	#define NUMBER_LEFT                  (172) // number
	#define NUMBER_TOP                   (213)
	#define NUMBER_RIGHT                 (210)
	#define NUMBER_BOTTOM                (267)

	// %  rectangle
	#define PERCENT_LEFT                 (248) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (213)
	#define PERCENT_RIGHT                (296)
	#define PERCENT_BOTTOM               (267)

	// top animation part
	#define TOP_ANIMATION_LEFT           (172) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (307)
	#define TOP_ANIMATION_BOTTOM         (124)

	// for old animation
	#define BAR_LEFT            (184)
	#define BAR_TOP             (227)
	#define BAR_RIGHT           (294)
	#define BAR_BOTTOM          (437)

#elif defined(QHD) || defined(CU_QHD) || defined(CMCC_QHD)
	// qhd 540*960

	// battery capacity rectangle
	#define CAPACITY_LEFT                (202) // battery capacity center
	#define CAPACITY_TOP                 (410)
	#define CAPACITY_RIGHT               (337)
	#define CAPACITY_BOTTOM              (626)

	// first number rectangle
	#define NUMBER_LEFT                  (202) // number
	#define NUMBER_TOP                   (266)
	#define NUMBER_RIGHT                 (240)
	#define NUMBER_BOTTOM                (320)

	// %  rectangle
	#define PERCENT_LEFT                 (278) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (266)
	#define PERCENT_RIGHT                (326)
	#define PERCENT_BOTTOM               (320)

	// top animation part
	#define TOP_ANIMATION_LEFT           (202) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (337)
	#define TOP_ANIMATION_BOTTOM         (124)

	// for old animation
	#define BAR_LEFT            (235)
	#define BAR_TOP             (179)
	#define BAR_RIGHT           (305)
	#define BAR_BOTTOM          (340)

#elif defined(WVGA) || defined(CU_WVGA) || defined(CMCC_WVGA)
	// default wvga 480*800

	// battery capacity rectangle
	#define CAPACITY_LEFT                (172) // battery capacity center
	#define CAPACITY_TOP                 (330)
	#define CAPACITY_RIGHT               (307)
	#define CAPACITY_BOTTOM              (546)

	// first number rectangle
	#define NUMBER_LEFT                  (178) // number
	#define NUMBER_TOP                   (190)
	#define NUMBER_RIGHT                 (216)
	#define NUMBER_BOTTOM                (244)

	// %  rectangle
	#define PERCENT_LEFT                 (254) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (190)
	#define PERCENT_RIGHT                (302)
	#define PERCENT_BOTTOM               (244)

	// top animation part
	#define TOP_ANIMATION_LEFT           (172) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (307)
	#define TOP_ANIMATION_BOTTOM         (124)

	// for old animation
	#define BAR_LEFT            (209)
	#define BAR_TOP             (149)
	#define BAR_RIGHT           (271)
	#define BAR_BOTTOM          (282)

#elif defined(HVGA) || defined(CU_HVGA) || defined(CMCC_HVGA)

	// hvga 320*480

	// battery capacity rectangle
	#define CAPACITY_LEFT                (109) // battery capacity center
	#define CAPACITY_TOP                 (189)
	#define CAPACITY_RIGHT               (211)
	#define CAPACITY_BOTTOM              (350)

	// first number rectangle
	#define NUMBER_LEFT                  (126) // number
	#define NUMBER_TOP                   (95)
	#define NUMBER_RIGHT                 (153)
	#define NUMBER_BOTTOM                (131)

	// %  rectangle
	#define PERCENT_LEFT                 (180) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (95)
	#define PERCENT_RIGHT                (212)
	#define PERCENT_BOTTOM               (131)

	// top animation part
	#define TOP_ANIMATION_LEFT           (109) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (211)
	#define TOP_ANIMATION_BOTTOM         (118)

	// for old animation
	#define BAR_LEFT            (129)
	#define BAR_TOP             (128)
	#define BAR_RIGHT           (190)
	#define BAR_BOTTOM          (245)
#elif defined(QVGA) || defined(CU_QVGA) || defined(CMCC_QVGA)

	// hvga 320*480

	// battery capacity rectangle
	#define CAPACITY_LEFT                (82) // battery capacity center
	#define CAPACITY_TOP                 (124)
	#define CAPACITY_RIGHT               (158)
	#define CAPACITY_BOTTOM              (241)

	// first number rectangle
	#define NUMBER_LEFT                  (93) // number
	#define NUMBER_TOP                   (50)
	#define NUMBER_RIGHT                 (109)
	#define NUMBER_BOTTOM                (73)

	// %  rectangle
	#define PERCENT_LEFT                 (125) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (50)
	#define PERCENT_RIGHT                (145)
	#define PERCENT_BOTTOM               (73)

	// top animation part
	#define TOP_ANIMATION_LEFT           (82) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (158)
	#define TOP_ANIMATION_BOTTOM         (113)

	// for old animation
	#define BAR_LEFT            (97)
	#define BAR_TOP             (96)
	#define BAR_RIGHT           (140)
	#define BAR_BOTTOM          (184)
	
#elif defined(WSVGA)
	// wsvga 600*1024

	// battery capacity rectangle
	#define CAPACITY_LEFT                (233) // battery capacity center //JRD Dong Jianfei modify on 2013-11-04.
	#define CAPACITY_TOP                 (442)
	#define CAPACITY_RIGHT               (368)
	#define CAPACITY_BOTTOM              (658)

	// first number rectangle
	#define NUMBER_LEFT                  (250) // number
	#define NUMBER_TOP                   (300)
	#define NUMBER_RIGHT                 (288)
	#define NUMBER_BOTTOM                (354)

	// %  rectangle
	#define PERCENT_LEFT                 (326) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (300)
	#define PERCENT_RIGHT                (374)
	#define PERCENT_BOTTOM               (354)

	// top animation part
	#define TOP_ANIMATION_LEFT           (233) // top animation //JRD Dong Jianfei modify on 2013-11-04.
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (368)
	#define TOP_ANIMATION_BOTTOM         (124)

	// for old animation
	#define BAR_LEFT            (260)
	#define BAR_TOP             (190)
	#define BAR_RIGHT           (338)
	#define BAR_BOTTOM          (360)

#elif defined(WXGANL)
	// wxvgnl 1280*800

	// battery capacity rectangle
	#define CAPACITY_LEFT                (278) // battery capacity center
	#define CAPACITY_TOP                 (556)
	#define CAPACITY_RIGHT               (441)
	#define CAPACITY_BOTTOM              (817)

	#define NUMBER_LEFT                  (290) // number
	#define NUMBER_TOP                   (386)
	#define NUMBER_RIGHT                 (335)
	#define NUMBER_BOTTOM                (450)

	#define PERCENT_LEFT                 (380) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (386)
	#define PERCENT_RIGHT                (437)
	#define PERCENT_BOTTOM               (450)

	#define TOP_ANIMATION_LEFT           (278) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (441)
	#define TOP_ANIMATION_BOTTOM         (124)

	// for old animation
	#define BAR_LEFT            (525)
	#define BAR_TOP             (250)
	#define BAR_RIGHT           (755)
	#define BAR_BOTTOM          (640)
	
#elif defined(WXGA)
	// wxga 800*1280

	// battery capacity rectangle
	#define CAPACITY_LEFT                (318) // battery capacity center yuanding modify 286->318
	#define CAPACITY_TOP                 (556) //yuanding modify 330->556
	#define CAPACITY_RIGHT               (481) //yuanding modify 421->481
	#define CAPACITY_BOTTOM              (815)  //yuanding modify 874->815

	#define NUMBER_LEFT                  (345) // number //yuanding modify 178->345
	#define NUMBER_TOP                   (385)           //yuanding modify 190->385
	#define NUMBER_RIGHT                 (390)           //yuanding modify 216->390
	#define NUMBER_BOTTOM                (449)           //yuanding modify 244->449

	#define PERCENT_LEFT                 (435) // percent number_left + 2*number_width //yuanding modify 254->435
	#define PERCENT_TOP                  (385)          //yuanding modify 190->385
	#define PERCENT_RIGHT                (492)          //yuanding modify 302->492 
	#define PERCENT_BOTTOM               (449)          //yuanding modify 244->449

	#define TOP_ANIMATION_LEFT           (318) // top animation   //yuanding modify 286->318
	#define TOP_ANIMATION_TOP            (100)                     //yuanding modify 100->100
	#define TOP_ANIMATION_RIGHT          (481)                     //yuanding modify 421->481
	#define TOP_ANIMATION_BOTTOM         (129)                     //yuanding modify 124->129

	// for old animation
	#define BAR_LEFT            (348)
	#define BAR_TOP             (238)
	#define BAR_RIGHT           (453)
	#define BAR_BOTTOM          (452)
	
#elif defined(XGA)
	// xga 768*1024

	// battery capacity rectangle
	#define CAPACITY_LEFT                (316) // battery capacity center
	#define CAPACITY_TOP                 (442)
	#define CAPACITY_RIGHT               (451)
	#define CAPACITY_BOTTOM              (658)

	#define NUMBER_LEFT                  (338) // number
	#define NUMBER_TOP                   (300)
	#define NUMBER_RIGHT                 (376)
	#define NUMBER_BOTTOM                (354)

	#define PERCENT_LEFT                 (414) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (300)
	#define PERCENT_RIGHT                (462)
	#define PERCENT_BOTTOM               (354)

	#define TOP_ANIMATION_LEFT           (316) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (451)
	#define TOP_ANIMATION_BOTTOM         (124)

	// for old animation
	#define BAR_LEFT            (334)
	#define BAR_TOP             (243)
	#define BAR_RIGHT           (434)
	#define BAR_BOTTOM          (463)
	
#elif defined(XGANL)
	// xganl 1024*768

	// battery capacity rectangle
	#define CAPACITY_LEFT                (444) // battery capacity center
	#define CAPACITY_TOP                 (314)	
	#define CAPACITY_RIGHT               (579)
	#define CAPACITY_BOTTOM              (530)

	#define NUMBER_LEFT                  (467) // number
	#define NUMBER_TOP                   (170)
	#define NUMBER_RIGHT                 (505)
	#define NUMBER_BOTTOM                (224)

	#define PERCENT_LEFT                 (543) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (170)
	#define PERCENT_RIGHT                (591)
	#define PERCENT_BOTTOM               (224)

	#define TOP_ANIMATION_LEFT           (444) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (579)
	#define TOP_ANIMATION_BOTTOM         (124)

	// for old animation
	#define BAR_LEFT            (486)
	#define BAR_TOP             (292)
	#define BAR_RIGHT           (590)
	#define BAR_BOTTOM          (506) 

#else 

	// default wvga 480*800

	// battery capacity rectangle
	#define CAPACITY_LEFT                (172) // battery capacity center
	#define CAPACITY_TOP                 (330)
	#define CAPACITY_RIGHT               (307)
	#define CAPACITY_BOTTOM              (546)

	// first number rectangle
	#define NUMBER_LEFT                  (178) // number
	#define NUMBER_TOP                   (190)
	#define NUMBER_RIGHT                 (216)
	#define NUMBER_BOTTOM                (244)

	// %  rectangle
	#define PERCENT_LEFT                 (254) // percent number_left + 2*number_width
	#define PERCENT_TOP                  (190)
	#define PERCENT_RIGHT                (302)
	#define PERCENT_BOTTOM               (244)

	// top animation part
	#define TOP_ANIMATION_LEFT           (172) // top animation
	#define TOP_ANIMATION_TOP            (100)
	#define TOP_ANIMATION_RIGHT          (307)
	#define TOP_ANIMATION_BOTTOM         (124)

	// for old animation
	#define BAR_LEFT            (209)
	#define BAR_TOP             (149)
	#define BAR_RIGHT           (271)
	#define BAR_BOTTOM          (282)

#endif

/* The option of new charging animation */
#define ANIMATION_NEW

#endif // __CUST_DISPLAY_H__
