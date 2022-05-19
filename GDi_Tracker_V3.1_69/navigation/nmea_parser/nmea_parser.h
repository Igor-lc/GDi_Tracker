


#ifndef _NMEA_PARSER_H
#define _NMEA_PARSER_H


#define NMEAP_MAX_SENTENCES            20
#define NMEAP_MAX_SENTENCE_NAME_LENGTH 7
#define NMEAP_MIN_SENTENCE_NAME_LENGTH 5
#define NMEAP_MAX_SENTENCE_LENGTH      100
#define NMEAP_MAX_TOKENS               24

typedef enum
{
	NMEAP_UNKNOWN_ID=0,
	NMEAP_GPGGA,//1
  NMEAP_GxRMC ,//2
  NMEAP_GxGSA ,//3
  NMEAP_GxGSV ,//4
  NMEAP_GxGLL ,//5
  NMEAP_PMTK010 ,//6
  NMEAP_GPTXT   ,//7
  NMEAP_GPVTG   ,//8
  NMEAP_PSTMTG,//9
}NmeaParserIdEnum;



#define HEXTOBIN(ch) ((ch <= '9') ? ch - '0' : ch - ('A' - 10))

struct nmeap_context;
struct nmeap_sentence;

typedef NmeaParserIdEnum (*nmeap_sentence_parser_t)(struct nmeap_context *context,struct nmeap_sentence *sentence);
typedef void (*nmeap_callout_t)(struct nmeap_context *context,void *sentence_data,void *user_data);



typedef struct nmeap_sentence 
{
    char                    name[NMEAP_MAX_SENTENCE_NAME_LENGTH + 1];
	  int                     id;
    nmeap_sentence_parser_t parser;
    nmeap_callout_t         callout;
    void                   *data;
	  int                    err_crc;
} nmeap_sentence_t;

typedef enum
{
	LOOKING_FOR_SOF,//0
	LOOKING_FOR_SENTENCE_ID,//1
	LOOKING_FOR_CR_OR_CRC_INDICATOR,//2
	 LOOKING_FOR_FIRST_CRC_CHAR,//3
	 LOOKING_FOR_SECOND_CRC_CHAR,//4
	 LOOKING_FOR_CR,//5
	 LOOKING_FOR_LF,//6
}ParserStateEnum;

struct nmeap_context 
{
	/** support up to 8 sentences */
	nmeap_sentence_t sentence[NMEAP_MAX_SENTENCES];		/* sentence descriptors */
	int              sentence_count;						/* number of initialized descriptors */
	
	/** sentence input buffer */
	char             input[NMEAP_MAX_SENTENCE_LENGTH + 1];	 /* input line buffer */
	int              input_count;	                        /* index into 'input */
	ParserStateEnum  state;	                        /* current lexical scanner state */
	char             input_s_name[NMEAP_MAX_SENTENCE_LENGTH+1];                        /* sentence name */
	char             in_crc; 			                        /* input checksum    */
	char             calc_crc; 			                        /* computed checksum */
	
	/* tokenization */
	char            *token[NMEAP_MAX_TOKENS];              /* list of delimited tokens */
	int              tokens;							     /* list of tokens */
	
	/** errors and debug. optimize these as desired */
	unsigned long    msgs;    /* count of good messages */
	unsigned long    err_header; /* header error */							
	unsigned long    err_ovr; /* overrun error */
	unsigned long    err_unk; /* unknown error */
	unsigned long    err_id;  /* bad character in id */
	unsigned long    err_crc; /* bad checksum */
	unsigned long    err_cr; /* expecting cr, got something else */
	unsigned long    err_lf; /* expecting cr, got something else */
	char             debug_input[NMEAP_MAX_SENTENCE_LENGTH + 1];	 /* input line buffer for debug */
	
	/** opaque user data */
	void *user_data;
};

typedef struct nmeap_context NmeaParserType;

extern NmeaParserIdEnum nmeap_parse(NmeaParserType *context,char ch);
extern NmeaParserIdEnum nmeap_parseBuffer(NmeaParserType *context,const char *buffer,int *length);
extern int nmeap_addParser(NmeaParserType *context,const char *sentence_name,nmeap_sentence_parser_t sentence_parser,nmeap_callout_t sentence_callout,void *sentence_data);
extern void nmeap_init(NmeaParserType *context,void *user_data);

#endif

