


#define NMEAP_MAX_SENTENCE_NAME_LENGTH 5
#define NMEAP_MAX_SENTENCES            8
#define NMEAP_MAX_SENTENCE_LENGTH      255
#define NMEAP_MAX_TOKENS               24
#define NMEAP_GPGGA 1
#define NMEAP_GPRMC 2


struct nmeap_context;
struct nmeap_sentence;
typedef struct nmeap_context nmeap_context_t;
typedef struct nmeap_rmc nmeap_rmc_t;

typedef int (*nmeap_sentence_parser_t)(struct nmeap_context *context,struct nmeap_sentence *sentence);
typedef void (*nmeap_callout_t)(struct nmeap_context *context,void *sentence_data,void *user_data);

typedef struct nmeap_sentence 
{
    char                    name[NMEAP_MAX_SENTENCE_NAME_LENGTH + 1];
	int                     id;
    nmeap_sentence_parser_t parser;
    nmeap_callout_t         callout;
    void                   *data;
} nmeap_sentence_t;

/** 
 * parser context
 */
struct nmeap_context {
	/** support up to 8 sentences */
	nmeap_sentence_t sentence[NMEAP_MAX_SENTENCES];		/* sentence descriptors */
	int              sentence_count;						/* number of initialized descriptors */
	
	/** sentence input buffer */
	char             input[NMEAP_MAX_SENTENCE_LENGTH + 1];	 /* input line buffer */
	int              input_count;	                        /* index into 'input */
	int              input_state;	                        /* current lexical scanner state */
	char             input_name[6];                        /* sentence name */
	char             icks; 			                        /* input checksum    */
	char             ccks; 			                        /* computed checksum */
	
	/* tokenization */
	char            *token[NMEAP_MAX_TOKENS];              /* list of delimited tokens */
	int              tokens;							     /* list of tokens */
	
	/** errors and debug. optimize these as desired */
	unsigned long    msgs;    /* count of good messages */
	unsigned long    err_hdr; /* header error */							
	unsigned long    err_ovr; /* overrun error */
	unsigned long    err_unk; /* unknown error */
	unsigned long    err_id;  /* bad character in id */
	unsigned long    err_cks; /* bad checksum */
	unsigned long    err_crl; /* expecting cr or lf, got something else */
	char             debug_input[NMEAP_MAX_SENTENCE_LENGTH + 1];	 /* input line buffer for debug */
	
	/** opaque user data */
	void *user_data;
};












