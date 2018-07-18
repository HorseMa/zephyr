#include "mymath.h"
#include <stdlib.h>
#include <stdint.h>

static uint32_t _sqrt_(uint64_t a)
{
        int i;
        uint64_t res;
        uint64_t remain;

        res = remain = 0ull;

        //之前整数平方根被直接优化，我们只需要求49位或者50位整数的平方根
        for(i=48;i>=0;i-=2) {//这里之前是46,改成48
                remain = (remain<<2)|((a&(3ull<<i))>>i);
                if(((res<<2)|1ull) <= remain) {
                        remain = remain - ((res<<2)|1ull);
                        res = (res<<1)|1ull;
                } else {
                        res <<= 1;
                }
        }

        return (uint32_t)res;
}

float mysqrtf(float f)
{
        union {
                float f;
                uint32_t u;
        } n;
        uint32_t N,A;
        int _N, i;
        uint64_t _A;

        n.f = f;
        if(n.u == 0x80000000 || n.u == 0x00000000) /* 0.0/-0.0 */
                return n.f;
        N = (n.u&(0xff<<23))>>23;
        if(N==0xff||(n.u&0x80000000)) { /* inf/-inf/nan/  f < 0.0*/
                n.u = 0x7fc00000; /* nan */
                return n.f;
        }
        if(N!=0x0) { /* 用科学计数法表示的规格数 */
                A = (n.u&0x7fffff)|0x800000;
                _N = (int)N - 127;
                if(N&0x1) {
                        _A = (uint64_t)A<<25;
                } else {
                        _A = (uint64_t)A<<26;
                        _N--;
                }
        } else { //A*2^(-149)这种表示方式的浮点数
                //还是需要找最高位
                for(i=22;;i--)
                        if(n.u&((0x1)<<i))
                                break;
                //然后需要移位，要区分奇数和偶数
                if(i&0x1) {
                        _N = i-149;
                        _A = (uint64_t)n.u << (48-i);
                } else {
                        _N = i-150;
                        _A = (uint64_t)n.u << (49-i);
                }
        }
        //小数部分
        A = _sqrt_(_A);
        //四舍五入
        A = (A+(A&0x1))>>1;
        //指数部分
        N = (uint32_t)(_N/2+127);
        //得到结果
        n.u = (A&0x7fffff)|(N<<23);
        return n.f;
}

double pow_i(double num,int n)//计算num的n次幂，其中n为整数
{
   double powint=1;
   int i;
   for(i=1;i<=n;i++) powint*=num;
   return powint;
}//计算num的n次幂，其中n为整数已验证

double pow_f(double num,double m)//计算num的m次幂，num和m可为双精度，num大于零
{
    int i,j;
    double powf=0,x,tmpm=1;
    x=num-1;
    for(i=1;tmpm>1e-12 || tmpm<-1e-12;i++)//当tmpm不在次范围时，停止循环,范围可改
           {
       for(j=1,tmpm=1;j<=i;j++)
            tmpm*=(m-j+1)*x/j;
            powf+=tmpm;
           }
    return powf+1;
}
double pow_ff(double num,double m)//调用pow_f()和pow_i(),计算num的m次幂,是计算幂的入口
{
    if(num==0 && m!=0)
        return 0;//若num为0，则返回0
    else if(num==0 && m==0)
         return 1;// 若num和m都为0，则返回1
    else if(num<0 && m-(int)(m)!=0)
        return 0;//若num为负，且m不为整数数，则出错，返回0
    if(num>2)//把底数大于2的情况转为(1/num)^-m计算
    {
        num=1/num;
        m=-m;
    }
    if(m<0)
        return 1/pow_ff(num,-m);//把指数小于0的情况转为1/num^-m计算
    if(m-(int)(m)==0)
        return pow_i(num,m);/*当指数为浮点数是，分成整数和小数分别求幂，这是因为但底数较小式，用pow_f直接求幂误差大，所以分为指数的整数部分用pow_i,小数部分用pow_f求.*/
    else
        return pow_f(num,m-(int)(m))*pow_i(num,(int)(m));
    return pow_f(num,m);
}
