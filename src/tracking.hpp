
#define __OPENCV_TRACKING_HPP__

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


/****************************************************************************************\
*                                  Motion Analysis                                       *
\****************************************************************************************/



#define  CV_LKFLOW_PYR_A_READY       1
#define  CV_LKFLOW_PYR_B_READY       2
#define  CV_LKFLOW_INITIAL_GUESSES   4
#define  CV_LKFLOW_GET_MIN_EIGENVALS 8


namespace cv
{

//! updates motion history image using the current silhouette
CV_EXPORTS_W void updateMotionHistory( InputArray silhouette, InputOutputArray mhi,
                                       double timestamp, double duration );

}

/* motion templates */
CV_IMPL void
cvUpdateMotionHistory( const void* silhouette, void* mhimg,
                       double timestamp, double mhi_duration )
{
    CvMat  silhstub, *silh = cvGetMat(silhouette, &silhstub);
    CvMat  mhistub, *mhi = cvGetMat(mhimg, &mhistub);

    if( !CV_IS_MASK_ARR( silh ))
        CV_Error( CV_StsBadMask, "" );

    if( CV_MAT_TYPE( mhi->type ) != CV_32FC1 )
        CV_Error( CV_StsUnsupportedFormat, "" );

    if( !CV_ARE_SIZES_EQ( mhi, silh ))
        CV_Error( CV_StsUnmatchedSizes, "" );

    CvSize size( mhi->width,mhi->height );

    if( CV_IS_MAT_CONT( mhi->type & silh->type ))
    {
        size.width *= size.height;
        size.height = 1;
    }

    float ts = (float)timestamp;
    float delbound = (float)(timestamp - mhi_duration);
    int x, y;
#if CV_SSE2
    volatile bool useSIMD = cv::checkHardwareSupport(CV_CPU_SSE2);
#endif

    for( y = 0; y < size.height; y++ )
    {
        const uchar* silhData = silh->data.ptr + silh->step*y;
        float* mhiData = (float*)(mhi->data.ptr + mhi->step*y);
        x = 0;

#if CV_SSE2
        if( useSIMD )
        {
            __m128 ts4 = _mm_set1_ps(ts), db4 = _mm_set1_ps(delbound);
            for( ; x <= size.width - 8; x += 8 )
            {
                __m128i z = _mm_setzero_si128();
                __m128i s = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i*)(silhData + x)), z);
                __m128 s0 = _mm_cvtepi32_ps(_mm_unpacklo_epi16(s, z)), s1 = _mm_cvtepi32_ps(_mm_unpackhi_epi16(s, z));
                __m128 v0 = _mm_loadu_ps(mhiData + x), v1 = _mm_loadu_ps(mhiData + x + 4);
                __m128 fz = _mm_setzero_ps();

                v0 = _mm_and_ps(v0, _mm_cmpge_ps(v0, db4));
                v1 = _mm_and_ps(v1, _mm_cmpge_ps(v1, db4));

                __m128 m0 = _mm_and_ps(_mm_xor_ps(v0, ts4), _mm_cmpneq_ps(s0, fz));
                __m128 m1 = _mm_and_ps(_mm_xor_ps(v1, ts4), _mm_cmpneq_ps(s1, fz));

                v0 = _mm_xor_ps(v0, m0);
                v1 = _mm_xor_ps(v1, m1);

                _mm_storeu_ps(mhiData + x, v0);
                _mm_storeu_ps(mhiData + x + 4, v1);
            }
        }
#endif

        for( ; x < size.width; x++ )
        {
            float val = mhiData[x];
            val = silhData[x] ? ts : val < delbound ? 0 : val;
            mhiData[x] = val;
        }
    }
}

void cv::updateMotionHistory( InputArray _silhouette, InputOutputArray _mhi,
                              double timestamp, double duration )
{
    Mat silhouette = _silhouette.getMat();
    CvMat c_silhouette = silhouette, c_mhi = _mhi.getMat();
    cvUpdateMotionHistory( &c_silhouette, &c_mhi, timestamp, duration );
}



