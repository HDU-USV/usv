/*******************************************************************************
* 文件名称：filter.c
* 摘    要：二阶IIR低通滤波器
*           使用时，需分别定义IIR_coeff_Typedef结构体，保存参数
*           一阶低通滤波器
* 当前版本：
* 作    者：across
* 日    期：2019/06/14
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/

#include "Filter.h"
#include "User_Library.h"
#include "math.h"

// LowPassFilter2p 

/*
* @brief  计算IIR滤波器参数
* @param  返回的IIR_coeff_Typedef结构体，采样频率，带宽
* @note		
* @retval void
* @author across
*/
void cal_iir_coeff(IIR_coeff_Typedef *coeff,float fs, float fc)
{		
  float fr =0;
  float ohm =0;
  float c =0;
	
	if (fc <= 0.0f) {
			// no filtering
			return;
	}
  
  fr= fs/fc;
  ohm=tanf(M_PI_F/fr);
  c=1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;
  
  coeff->fc = fc;
  
  coeff->b0 = ohm*ohm/c;
  coeff->b1 = 2.0f*coeff->b0;
  coeff->b2 = coeff->b0;
  coeff->a1 = 2.0f*(ohm*ohm-1.0f)/c;
  coeff->a2 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
}

/*
* @brief  滤波函数
* @param  IIR_coeff_Typedef结构体，待滤波的原始数据
* @note		
* @retval 滤波后的数据
* @author across
*/
float get_iir_output(IIR_coeff_Typedef* coeff,float sample)
{
  if (coeff->fc <= 0.0f) {
		// no filtering
    return sample;
  }

	float y_0 = sample - coeff->y_1 * coeff->a1 - coeff->y_2 * coeff->a2;    
	float output = y_0 * coeff->b0 + coeff->y_1 * coeff->b1 + coeff->y_2 * coeff->b2;    
	coeff->y_2 = coeff->y_1;
	coeff->y_1 = y_0;
	return output;
	
}


/*
* @brief  一阶低通滤波系数计算
* @param  截止频率  采样周期
* @note		
* @retval 滤波器系数
* @author across
*/
float compute_alpha(float cutoff_freq, float dt)
{
		float alpha;
	
		if (cutoff_freq <= 0.0f || dt <= 0.0f) {
				alpha = 1.0f;
		}
		else
		{
				float rc = 1.0f / (M_2PI * cutoff_freq);
		    alpha = constrain_float(dt/(dt+rc), 0.0f, 1.0f);
		}
		
		return alpha;
}






