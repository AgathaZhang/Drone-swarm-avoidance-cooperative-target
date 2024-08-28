#ifndef _MD5_H
#define _MD5_H


typedef struct
{  
    unsigned int count[2];  
    unsigned int state[4];  
    unsigned char buffer[64];     
}md5Ctx_t;  

                       
#define F(x,y,z) ((x & y) | (~x & z))  
#define G(x,y,z) ((x & z) | (y & ~z))  
#define H(x,y,z) (x^y^z)  
#define I(x,y,z) (y ^ (x | ~z))  
#define ROTATE_LEFT(x,n) ((x << n) | (x >> (32-n)))  


#define FF(a,b,c,d,x,s,ac)\
          { \
						a += F(b,c,d) + x + ac; \
						a = ROTATE_LEFT(a,s); \
						a += b; \
          }  
#define GG(a,b,c,d,x,s,ac) \
          { \
						a += G(b,c,d) + x + ac; \
						a = ROTATE_LEFT(a,s); \
						a += b; \
          }  
#define HH(a,b,c,d,x,s,ac) \
          { \
						a += H(b,c,d) + x + ac; \
						a = ROTATE_LEFT(a,s); \
						a += b; \
          } 
					
					#define II(a,b,c,d,x,s,ac) \
          { \
						a += I(b,c,d) + x + ac; \
						a = ROTATE_LEFT(a,s); \
						a += b; \
          }    


					
void MD5Init(md5Ctx_t *context);
void MD5Update(md5Ctx_t *context,unsigned char *input,unsigned int inputlen);  
void MD5Final(md5Ctx_t *context,unsigned char digest[16]);  
void MD5Transform(unsigned int state[4],unsigned char block[64]);  
void MD5EncodeIntToChar(unsigned char *output,unsigned int *input,unsigned int len);  
void MD5DecodeCharToInt(unsigned int *output,unsigned char *input,unsigned int len);

int md5Test(void);
void MDString (char *string, unsigned int stringLen, unsigned char digest[16]) ;

#endif




