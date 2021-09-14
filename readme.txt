Implementation of the following function in opencv


template<class CastOp, class VecOp> void
pyrDown_( const Mat& _src, Mat& _dst, int borderType )
{
    const int PD_SZ = 5;
    typedef typename CastOp::type1 WT;
    typedef typename CastOp::rtype T;

    CV_Assert( !_src.empty() );
    Size ssize = _src.size(), dsize = _dst.size();
    int cn = _src.channels();
    int bufstep = (int)alignSize(dsize.width*cn, 16);
    AutoBuffer<WT> _buf(bufstep*PD_SZ + 16);
    WT* buf = alignPtr((WT*)_buf, 16);
    int tabL[CV_CN_MAX*(PD_SZ+2)], tabR[CV_CN_MAX*(PD_SZ+2)];
    AutoBuffer<int> _tabM(dsize.width*cn);
    int* tabM = _tabM;
    WT* rows[PD_SZ];
    CastOp castOp;
    VecOp vecOp;
    char path[64] = { 0 };
    int pyramid_level = 0;
    sprintf(path, "hor%d.txt", pyramid_level);
    fopen_s(&f_hor, path, "wb");
    sprintf(path, "ver%d.txt", pyramid_level);
    fopen_s(&f_ver, "ver.txt", "wb");

    CV_Assert( ssize.width > 0 && ssize.height > 0 &&
               std::abs(dsize.width*2 - ssize.width) <= 2 &&
               std::abs(dsize.height*2 - ssize.height) <= 2 );
    int k, x, sy0 = -PD_SZ/2, sy = sy0, width0 = std::min((ssize.width-PD_SZ/2-1)/2 + 1, dsize.width);

    for( x = 0; x <= PD_SZ+1; x++ )
    {
        int sx0 = borderInterpolate(x - PD_SZ/2, ssize.width, borderType)*cn;
        int sx1 = borderInterpolate(x + width0*2 - PD_SZ/2, ssize.width, borderType)*cn;
        for( k = 0; k < cn; k++ )
        {
            tabL[x*cn + k] = sx0 + k;
            tabR[x*cn + k] = sx1 + k;
        }
    }

    ssize.width *= cn;
    dsize.width *= cn;
    width0 *= cn;

    for( x = 0; x < dsize.width; x++ )
        tabM[x] = (x/cn)*2*cn + x % cn;

    for( int y = 0; y < dsize.height; y++ )
    {
        T* dst = _dst.ptr<T>(y);
        WT *row0, *row1, *row2, *row3, *row4;

        // fill the ring buffer (horizontal convolution and decimation)
        for( ; sy <= y*2 + 2; sy++ )
        {
            WT* row = buf + ((sy - sy0) % PD_SZ)*bufstep;
            int _sy = borderInterpolate(sy, ssize.height, borderType);
            const T* src = _src.ptr<T>(_sy);
            int limit = cn;
            const int* tab = tabL;

            for( x = 0;;)
            {
                for( ; x < limit; x++ )
                {
                    row[x] = src[tab[x+cn*2]]*6 + (src[tab[x+cn]] + src[tab[x+cn*3]])*4 +
                        src[tab[x]] + src[tab[x+cn*4]];
                    if (sy >= 0)
                        Log_hor("y=%d x=%d conv=%d\n",sy,x,row[x]);
                }

                if( x == dsize.width )
                    break;

                if( cn == 1 )
                {
                    for (; x < width0; x++){
                        row[x] = src[x * 2] * 6 + (src[x * 2 - 1] + src[x * 2 + 1]) * 4 +
                            src[x * 2 - 2] + src[x * 2 + 2];
                        if (sy >= 0)
                            Log_hor("y=%d x=%d conv=%d\n",sy, x, row[x]);
                    }

                }
                else if( cn == 3 )
                {
                    for( ; x < width0; x += 3 )
                    {
                        const T* s = src + x*2;
                        WT t0 = s[0]*6 + (s[-3] + s[3])*4 + s[-6] + s[6];
                        WT t1 = s[1]*6 + (s[-2] + s[4])*4 + s[-5] + s[7];
                        WT t2 = s[2]*6 + (s[-1] + s[5])*4 + s[-4] + s[8];
                        row[x] = t0; row[x+1] = t1; row[x+2] = t2;
                    }
                }
                else if( cn == 4 )
                {
                    for( ; x < width0; x += 4 )
                    {
                        const T* s = src + x*2;
                        WT t0 = s[0]*6 + (s[-4] + s[4])*4 + s[-8] + s[8];
                        WT t1 = s[1]*6 + (s[-3] + s[5])*4 + s[-7] + s[9];
                        row[x] = t0; row[x+1] = t1;
                        t0 = s[2]*6 + (s[-2] + s[6])*4 + s[-6] + s[10];
                        t1 = s[3]*6 + (s[-1] + s[7])*4 + s[-5] + s[11];
                        row[x+2] = t0; row[x+3] = t1;
                    }
                }
                else
                {
                    for( ; x < width0; x++ )
                    {
                        int sx = tabM[x];
                        row[x] = src[sx]*6 + (src[sx - cn] + src[sx + cn])*4 +
                            src[sx - cn*2] + src[sx + cn*2];
                    }
                }

                limit = dsize.width;
                tab = tabR - x;
            }
        }

        // do vertical convolution and decimation and write the result to the destination image
        for( k = 0; k < PD_SZ; k++ )
            rows[k] = buf + ((y*2 - PD_SZ/2 + k - sy0) % PD_SZ)*bufstep;
        row0 = rows[0]; row1 = rows[1]; row2 = rows[2]; row3 = rows[3]; row4 = rows[4];

        //x = vecOp(rows, dst, (int)_dst.step, dsize.width);
        x = 0;
        for (; x < dsize.width; x++){
            dst[x] = castOp(row2[x] * 6 + (row1[x] + row3[x]) * 4 + row0[x] + row4[x]);
            Log_ver("y=%d x=%d conv=%d cast=%d(0x%x)\n", y, x, row2[x] * 6 + (row1[x] + row3[x]) * 4 + row0[x] + row4[x]+128, dst[x], dst[x]);
        }
            
    }

    fclose(f_hor);
    fclose(f_ver);
}



1. Content
Folder "src" contains all source file.
Folder "tb" contains test bench file, ext_ram_32.v emulate ddr with axi3 interface.
Folder "pli_fputc" is verilog pli used to write output bin to file when run simulation.

How to use: 
Simulation: add all test bench and source code file to your simulation project source, for example, modelsim.
put the test file, orig_img1 to your simulation project folder.
then run, for example, for modelsim, run "vsim -pli pli_fputc.dll pyr_tb".
The output is out.yuv, and some log files.
