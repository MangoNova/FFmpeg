/*
 * H.26L/H.264/AVC/JVT/14496-10/... decoder
 * Copyright (c) 2003 Michael Niedermayer <michaelni@gmx.at>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#undef FUNC
#undef PIXEL_SHIFT

#if SIMPLE
#   define FUNC(n) AV_JOIN(n ## _simple_, BITS)
#   define PIXEL_SHIFT (BITS >> 4)
#else
#   define FUNC(n) n ## _complex
#   define PIXEL_SHIFT h->pixel_shift
#endif

#undef  CHROMA_IDC
#define CHROMA_IDC 1
#include "h264_mc_template.c"

#undef  CHROMA_IDC
#define CHROMA_IDC 2
#include "h264_mc_template.c"

// 定义一个函数，用于解码H.264的宏块
static av_noinline void FUNC(hl_decode_mb)(const H264Context *h, H264SliceContext *sl)
{
    // 获取宏块的坐标和类型
    const int mb_x    = sl->mb_x;
    const int mb_y    = sl->mb_y;
    const int mb_xy   = sl->mb_xy;
    //mb_xy 是什么，mb_xy是宏块的二维到一维的映射后索引？用于定位信息？
    const int mb_type = h->cur_pic.mb_type[mb_xy];

    // 定义Y, Cb, Cr的目标缓冲区
    uint8_t *dest_y, *dest_cb, *dest_cr;

    // 定义行大小和UV行大小
    int linesize, uvlinesize;

    // 定义其他变量
    int i, j;
    const int *block_offset = &h->block_offset[0];

    // 检查是否需要进行变换绕过
    // 在H.264编码中，qscale 是一个量化参数。当 qscale 为0时，通常表示正在使用无损编码，这可能需要变换绕过。
    // h->ps.sps->transform_bypass 检查序列参数集（SPS）中的变换绕过标志是否设置
    // 在 H.264 视频编码中，序列参数集（SPS）包含了对整个序列（即一系列连续的视频帧）有效的参数。这些参数包括像素尺寸、帧率、颜色空间等信息。
    const int transform_bypass = !SIMPLE && (sl->qscale == 0 && h->ps.sps->transform_bypass);
    // 定义IDCT添加函数
    void (*idct_add)(uint8_t *dst, int16_t *block, int stride);

    // 获取块的高度和是否为422色度格式
    const int block_h   = 16 >> h->chroma_y_shift;
    const int chroma422 = CHROMA422(h);

    /*计算Y, Cb, Cr的目标缓冲区地址,dest_y 是 Y 分量的目标缓冲区地址。
    它是通过将宏块的 x 坐标 (mb_x) 左移 PIXEL_SHIFT 位（相当于乘以 2^PIXEL_SHIFT），
    然后加上宏块的 y 坐标 (mb_y) 乘以每行的字节数 (sl->linesize)，最后乘以16得到的。这里乘以16是因为每个宏块包含16行像素。
    dest_cb 和 dest_cr 是 Cb 和 Cr 分量的目标缓冲区地址。它们的计算方式与 dest_y 类似，但是有两个主要的区别。
    首先，它们是通过将 mb_x 左移 PIXEL_SHIFT 位然后乘以8得到的，这是因为 Cb 和 Cr 分量的分辨率通常是 Y 分量的一半。
    其次，它们是通过将 mb_y 乘以 UV 分量每行的字节数 (sl->uvlinesize) 然后乘以 block_h 得到的，这是因为 UV 分量的垂直分辨率也可能是 Y 分量的一半。
    */
    dest_y  = h->cur_pic.f->data[0] + ((mb_x << PIXEL_SHIFT)     + mb_y * sl->linesize)  * 16;
    dest_cb = h->cur_pic.f->data[1] +  (mb_x << PIXEL_SHIFT) * 8 + mb_y * sl->uvlinesize * block_h;
    dest_cr = h->cur_pic.f->data[2] +  (mb_x << PIXEL_SHIFT) * 8 + mb_y * sl->uvlinesize * block_h;

    // 预取数据，提高数据访问效率，预取的实际上是字节流
    h->vdsp.prefetch(dest_y  + (sl->mb_x & 3) * 4 * sl->linesize   + (64 << PIXEL_SHIFT), sl->linesize,       4);
    h->vdsp.prefetch(dest_cb + (sl->mb_x & 7)     * sl->uvlinesize + (64 << PIXEL_SHIFT), dest_cr - dest_cb, 2);

    // 设置当前宏块的列表计数
    h->list_counts[mb_xy] = sl->list_count;

    // 如果是场编码的宏块
    if (!SIMPLE && MB_FIELD(sl)) {
        // 更新行大小和UV行大小
        linesize     = sl->mb_linesize = sl->linesize * 2;
        uvlinesize   = sl->mb_uvlinesize = sl->uvlinesize * 2;
        block_offset = &h->block_offset[48];

        // 如果是场的下半部分
        if (mb_y & 1) {
            // 更新Y, Cb, Cr的目标缓冲区地址
            dest_y  -= sl->linesize * 15;
            dest_cb -= sl->uvlinesize * (block_h - 1);
            dest_cr -= sl->uvlinesize * (block_h - 1);
        }

        // 如果是帧场自适应编码
        if (FRAME_MBAFF(h)) {
            int list;
            for (list = 0; list < sl->list_count; list++) {
                if (!USES_LIST(mb_type, list))
                    continue;
                if (IS_16X16(mb_type)) {
                    int8_t *ref = &sl->ref_cache[list][scan8[0]];
                    fill_rectangle(ref, 4, 4, 8, (16 + *ref) ^ (sl->mb_y & 1), 1);
                } else {
                    for (i = 0; i < 16; i += 4) {
                        int ref = sl->ref_cache[list][scan8[i]];
                        if (ref >= 0)
                            fill_rectangle(&sl->ref_cache[list][scan8[i]], 2, 2,
                                           8, (16 + ref) ^ (sl->mb_y & 1), 1);
                    }
                }
            }
        }
    } else {
        // 如果是帧编码的宏块，更新行大小和UV行大小
        linesize   = sl->mb_linesize   = sl->linesize;
        uvlinesize = sl->mb_uvlinesize = sl->uvlinesize;
    }

    // 如果是帧内PCM编码的宏块
    // 如果当前宏块是PCM宏块
    if (!SIMPLE && IS_INTRA_PCM(mb_type)) {
        // 获取亮度分量的位深度
        const int bit_depth = h->ps.sps->bit_depth_luma;

        // 如果需要进行像素位移
        if (PIXEL_SHIFT) {
            int j;
            GetBitContext gb;

            // 初始化比特流读取上下文
            init_get_bits(&gb, sl->intra_pcm_ptr,
                          ff_h264_mb_sizes[h->ps.sps->chroma_format_idc] * bit_depth);

            // 读取亮度分量的PCM值
            for (i = 0; i < 16; i++) {
                uint16_t *tmp_y = (uint16_t *)(dest_y + i * linesize);
                for (j = 0; j < 16; j++)
                    tmp_y[j] = get_bits(&gb, bit_depth);
            }

            // 如果不是灰度模式
            if (SIMPLE || !CONFIG_GRAY || !(h->flags & AV_CODEC_FLAG_GRAY)) {
                // 如果色度格式为单色
                if (!h->ps.sps->chroma_format_idc) {
                    // 设置色度分量的PCM值为中值
                    for (i = 0; i < block_h; i++) {
                        uint16_t *tmp_cb = (uint16_t *)(dest_cb + i * uvlinesize);
                        uint16_t *tmp_cr = (uint16_t *)(dest_cr + i * uvlinesize);
                        for (j = 0; j < 8; j++) {
                            tmp_cb[j] = tmp_cr[j] = 1 << (bit_depth - 1);
                        }
                    }
                } else {
                    // 读取Cb和Cr分量的PCM值
                    for (i = 0; i < block_h; i++) {
                        uint16_t *tmp_cb = (uint16_t *)(dest_cb + i * uvlinesize);
                        for (j = 0; j < 8; j++)
                            tmp_cb[j] = get_bits(&gb, bit_depth);
                    }
                    for (i = 0; i < block_h; i++) {
                        uint16_t *tmp_cr = (uint16_t *)(dest_cr + i * uvlinesize);
                        for (j = 0; j < 8; j++)
                            tmp_cr[j] = get_bits(&gb, bit_depth);
                    }
                }
            }
        } else {
            // 如果不需要进行像素位移，直接复制PCM值
            for (i = 0; i < 16; i++)
                memcpy(dest_y + i * linesize, sl->intra_pcm_ptr + i * 16, 16);

            // 如果不是灰度模式
            if (SIMPLE || !CONFIG_GRAY || !(h->flags & AV_CODEC_FLAG_GRAY)) {
                // 如果色度格式为单色
                if (!h->ps.sps->chroma_format_idc) {
                    // 设置色度分量的PCM值为中值
                    for (i = 0; i < 8; i++) {
                        memset(dest_cb + i * uvlinesize, 1 << (bit_depth - 1), 8);
                        memset(dest_cr + i * uvlinesize, 1 << (bit_depth - 1), 8);
                    }
                } else {
                    // 直接复制Cb和Cr分量的PCM值
                    const uint8_t *src_cb = sl->intra_pcm_ptr + 256;
                    const uint8_t *src_cr = sl->intra_pcm_ptr + 256 + block_h * 8;
                    for (i = 0; i < block_h; i++) {
                        memcpy(dest_cb + i * uvlinesize, src_cb + i * 8, 8);
                        memcpy(dest_cr + i * uvlinesize, src_cr + i * 8, 8);
                    }
                }
            }
        }
    } else {
        //帧内预测
        //Intra类型
    	//Intra4x4或者Intra16x16
        if (IS_INTRA(mb_type)) {
            if (sl->deblocking_filter)
                xchg_mb_border(h, sl, dest_y, dest_cb, dest_cr, linesize,
                               uvlinesize, 1, 0, SIMPLE, PIXEL_SHIFT);

            if (SIMPLE || !CONFIG_GRAY || !(h->flags & AV_CODEC_FLAG_GRAY)) {
                h->hpc.pred8x8[sl->chroma_pred_mode](dest_cb, uvlinesize);
                h->hpc.pred8x8[sl->chroma_pred_mode](dest_cr, uvlinesize);
            }
            //帧内预测-亮度
            hl_decode_mb_predict_luma(h, sl, mb_type, SIMPLE,
                                      transform_bypass, PIXEL_SHIFT,
                                      block_offset, linesize, dest_y, 0);

            if (sl->deblocking_filter)
                xchg_mb_border(h, sl, dest_y, dest_cb, dest_cr, linesize,
                               uvlinesize, 0, 0, SIMPLE, PIXEL_SHIFT);
        } else {
            //Inter类型
 
            //运动补偿
            if (chroma422) {
                FUNC(hl_motion_422)(h, sl, dest_y, dest_cb, dest_cr,
                              h->h264qpel.put_h264_qpel_pixels_tab,
                              h->h264chroma.put_h264_chroma_pixels_tab,
                              h->h264qpel.avg_h264_qpel_pixels_tab,
                              h->h264chroma.avg_h264_chroma_pixels_tab,
                              h->h264dsp.weight_h264_pixels_tab,
                              h->h264dsp.biweight_h264_pixels_tab);
            } else {
                //“*_put”处理单向预测，“*_avg”处理双向预测，“weight”处理加权预测
            	//h->qpel_put[16]包含了单向预测的四分之一像素运动补偿所有样点处理的函数
            	//两个像素之间横向的点（内插点和原始的点）有4个，纵向的点有4个，组合起来一共16个
            	//h->qpel_avg[16]情况也类似
                FUNC(hl_motion_420)(h, sl, dest_y, dest_cb, dest_cr,
                              h->h264qpel.put_h264_qpel_pixels_tab,
                              h->h264chroma.put_h264_chroma_pixels_tab,
                              h->h264qpel.avg_h264_qpel_pixels_tab,
                              h->h264chroma.avg_h264_chroma_pixels_tab,
                              h->h264dsp.weight_h264_pixels_tab,
                              h->h264dsp.biweight_h264_pixels_tab);
            }
        }
        //亮度的IDCT
        hl_decode_mb_idct_luma(h, sl, mb_type, SIMPLE, transform_bypass,
                               PIXEL_SHIFT, block_offset, linesize, dest_y, 0);
        //色度的IDCT（没有写在一个单独的函数中）
        // 如果不是灰度模式，并且色度分量的预测模式不为0
        if ((SIMPLE || !CONFIG_GRAY || !(h->flags & AV_CODEC_FLAG_GRAY)) &&
            (sl->cbp & 0x30)) {
            // 定义色度分量的目标缓冲区
            uint8_t *dest[2] = { dest_cb, dest_cr };

            // 如果需要进行变换绕过
            if (transform_bypass) {
                // 如果当前宏块是帧内预测宏块，并且编码配置文件的IDC为244，
                // 并且色度预测模式为垂直或水平
                if (IS_INTRA(mb_type) && h->ps.sps->profile_idc == 244 &&
                    (sl->chroma_pred_mode == VERT_PRED8x8 ||
                     sl->chroma_pred_mode == HOR_PRED8x8)) {
                    // 使用色度预测模式对应的预测函数进行预测
                    h->hpc.pred8x8_add[sl->chroma_pred_mode](dest[0],
                                                              block_offset + 16,
                                                              sl->mb + (16 * 16 * 1 << PIXEL_SHIFT),
                                                              uvlinesize);
                    h->hpc.pred8x8_add[sl->chroma_pred_mode](dest[1],
                                                              block_offset + 32,
                                                              sl->mb + (16 * 16 * 2 << PIXEL_SHIFT),
                                                              uvlinesize);
                } else {
                    // 否则，使用4x4的IDCT添加函数
                    idct_add = h->h264dsp.h264_add_pixels4_clear;

                    // 对Cb和Cr分量进行处理
                    for (j = 1; j < 3; j++) {
                        // 对每个4x4的子块进行处理
                        for (i = j * 16; i < j * 16 + 4; i++)
                            // 如果子块的非零系数计数不为0，或者子块的DCT系数不为0
                            if (sl->non_zero_count_cache[scan8[i]] ||
                                dctcoef_get(sl->mb, PIXEL_SHIFT, i * 16))
                                // 对子块进行IDCT并添加到目标缓冲区
                                idct_add(dest[j - 1] + block_offset[i],
                                         sl->mb + (i * 16 << PIXEL_SHIFT),
                                         uvlinesize);

                        // 如果是422色度格式
                        if (chroma422) {
                            // 对每个4x4的子块进行处理
                            for (i = j * 16 + 4; i < j * 16 + 8; i++)
                                // 如果子块的非零系数计数不为0，或者子块的DCT系数不为0
                                if (sl->non_zero_count_cache[scan8[i + 4]] ||
                                    dctcoef_get(sl->mb, PIXEL_SHIFT, i * 16))
                                    // 对子块进行IDCT并添加到目标缓冲区
                                    idct_add(dest[j - 1] + block_offset[i + 4],
                                             sl->mb + (i * 16 << PIXEL_SHIFT),
                                             uvlinesize);
                        }
                    }
                }
            
            } else {
                // 定义色度量化参数
                int qp[2];
                // 如果是422色度格式，色度量化参数需要加3
                if (chroma422) {
                    qp[0] = sl->chroma_qp[0] + 3;
                    qp[1] = sl->chroma_qp[1] + 3;
                } else {
                    qp[0] = sl->chroma_qp[0];
                    qp[1] = sl->chroma_qp[1];
                }
                // 如果Cb分量的DC块的非零系数计数不为0
                if (sl->non_zero_count_cache[scan8[CHROMA_DC_BLOCK_INDEX + 0]])
                    // 对Cb分量的DC块进行反量化和IDCT
                    h->h264dsp.h264_chroma_dc_dequant_idct(sl->mb + (16 * 16 * 1 << PIXEL_SHIFT),
                                                           h->ps.pps->dequant4_coeff[IS_INTRA(mb_type) ? 1 : 4][qp[0]][0]);
                // 如果Cr分量的DC块的非零系数计数不为0
                if (sl->non_zero_count_cache[scan8[CHROMA_DC_BLOCK_INDEX + 1]])
                    // 对Cr分量的DC块进行反量化和IDCT
                    h->h264dsp.h264_chroma_dc_dequant_idct(sl->mb + (16 * 16 * 2 << PIXEL_SHIFT),
                                                           h->ps.pps->dequant4_coeff[IS_INTRA(mb_type) ? 2 : 5][qp[1]][0]);
                // 对色度分量的所有8x8块进行IDCT并添加到目标缓冲区
                h->h264dsp.h264_idct_add8(dest, block_offset,
                                          sl->mb, uvlinesize,
                                          sl->non_zero_count_cache);
            }
        }
    }
}

#if !SIMPLE || BITS == 8

#undef  CHROMA_IDC
#define CHROMA_IDC 3
#include "h264_mc_template.c"

static av_noinline void FUNC(hl_decode_mb_444)(const H264Context *h, H264SliceContext *sl)
{
    const int mb_x    = sl->mb_x;
    const int mb_y    = sl->mb_y;
    const int mb_xy   = sl->mb_xy;
    const int mb_type = h->cur_pic.mb_type[mb_xy];
    uint8_t *dest[3];
    int linesize;
    int i, j, p;
    const int *block_offset = &h->block_offset[0];
    const int transform_bypass = !SIMPLE && (sl->qscale == 0 && h->ps.sps->transform_bypass);
    const int plane_count      = (SIMPLE || !CONFIG_GRAY || !(h->flags & AV_CODEC_FLAG_GRAY)) ? 3 : 1;

    for (p = 0; p < plane_count; p++) {
        dest[p] = h->cur_pic.f->data[p] +
                  ((mb_x << PIXEL_SHIFT) + mb_y * sl->linesize) * 16;
        h->vdsp.prefetch(dest[p] + (sl->mb_x & 3) * 4 * sl->linesize + (64 << PIXEL_SHIFT),
                         sl->linesize, 4);
    }

    // 保存当前宏块的参考帧列表计数
    h->list_counts[mb_xy] = sl->list_count;

    // 如果当前宏块是场模式
    if (!SIMPLE && MB_FIELD(sl)) {
        // 设置宏块的行大小和UV行大小为原来的两倍
        linesize     = sl->mb_linesize = sl->mb_uvlinesize = sl->linesize * 2;
        // 设置块偏移为48
        block_offset = &h->block_offset[48];
        // 如果宏块的y坐标为奇数
        if (mb_y & 1) // FIXME move out of this function?
            // 对每个平面，将目标地址向上偏移15行
            for (p = 0; p < 3; p++)
                dest[p] -= sl->linesize * 15;
        // 如果是帧场自适应模式
        if (FRAME_MBAFF(h)) {
            int list;
            // 对每个参考帧列表进行处理
            for (list = 0; list < sl->list_count; list++) {
                // 如果当前宏块不使用该参考帧列表，跳过
                if (!USES_LIST(mb_type, list))
                    continue;
                // 如果当前宏块是16x16模式
                if (IS_16X16(mb_type)) {
                    int8_t *ref = &sl->ref_cache[list][scan8[0]];
                    // 将参考帧索引加16并与宏块的y坐标进行异或，然后填充到参考帧缓存
                    fill_rectangle(ref, 4, 4, 8, (16 + *ref) ^ (sl->mb_y & 1), 1);
                } else {
                    // 如果当前宏块是其他模式
                    for (i = 0; i < 16; i += 4) {
                        int ref = sl->ref_cache[list][scan8[i]];
                        // 如果参考帧索引大于等于0
                        if (ref >= 0)
                            // 将参考帧索引加16并与宏块的y坐标进行异或，然后填充到参考帧缓存
                            fill_rectangle(&sl->ref_cache[list][scan8[i]], 2, 2,
                                           8, (16 + ref) ^ (sl->mb_y & 1), 1);
                    }
                }
            }
        }
    } else {
        // 如果当前宏块是帧模式，行大小和UV行大小不变
        linesize = sl->mb_linesize = sl->mb_uvlinesize = sl->linesize;
    }
        // 如果当前宏块类型不是简单类型，并且是PCM内部预测类型
        if (!SIMPLE && IS_INTRA_PCM(mb_type)) {
            // 如果需要进行像素位移
            if (PIXEL_SHIFT) {
                // 获取亮度的位深度
                const int bit_depth = h->ps.sps->bit_depth_luma;
                // 定义一个位流读取上下文
                GetBitContext gb;
                // 初始化位流读取上下文，设置位流数据和位流长度
                init_get_bits(&gb, sl->intra_pcm_ptr, 768 * bit_depth);

                // 对每个平面进行处理
                for (p = 0; p < plane_count; p++)
                    // 对每个16x16的宏块进行处理
                    for (i = 0; i < 16; i++) {
                        // 获取当前行的数据指针
                        uint16_t *tmp = (uint16_t *)(dest[p] + i * linesize);
                        // 对当前行的每个像素进行处理
                        for (j = 0; j < 16; j++)
                            // 从位流中读取像素值
                            tmp[j] = get_bits(&gb, bit_depth);
                    }
            } else {
                // 对每个平面进行处理
                for (p = 0; p < plane_count; p++)
                    // 对每个16x16的宏块进行处理
                    for (i = 0; i < 16; i++)
                        // 从PCM内部预测数据中复制数据到目标缓冲区
                        memcpy(dest[p] + i * linesize,
                               sl->intra_pcm_ptr + p * 256 + i * 16, 16);
            }
        } else {
            // 如果当前宏块类型是内部预测类型
            if (IS_INTRA(mb_type)) {
                // 如果需要进行去块效应滤波
                if (sl->deblocking_filter)
                    // 交换宏块边界数据
                    xchg_mb_border(h, sl, dest[0], dest[1], dest[2], linesize,
                                   linesize, 1, 1, SIMPLE, PIXEL_SHIFT);

                // 对每个平面进行处理
                for (p = 0; p < plane_count; p++)
                    // 对亮度进行预测解码
                    hl_decode_mb_predict_luma(h, sl, mb_type, SIMPLE,
                                              transform_bypass, PIXEL_SHIFT,
                                              block_offset, linesize, dest[p], p);

                // 如果需要进行去块效应滤波
                if (sl->deblocking_filter)
                    // 交换宏块边界数据
                    xchg_mb_border(h, sl, dest[0], dest[1], dest[2], linesize,
                                   linesize, 0, 1, SIMPLE, PIXEL_SHIFT);
            } else {
                // 对444色度格式的运动补偿进行处理
                FUNC(hl_motion_444)(h, sl, dest[0], dest[1], dest[2],
                          h->h264qpel.put_h264_qpel_pixels_tab,
                          h->h264chroma.put_h264_chroma_pixels_tab,
                          h->h264qpel.avg_h264_qpel_pixels_tab,
                          h->h264chroma.avg_h264_chroma_pixels_tab,
                          h->h264dsp.weight_h264_pixels_tab,
                          h->h264dsp.biweight_h264_pixels_tab);
            }

            // 对每个平面进行处理
            for (p = 0; p < plane_count; p++)
                // 对亮度进行IDCT解码
                hl_decode_mb_idct_luma(h, sl, mb_type, SIMPLE, transform_bypass,
                                       PIXEL_SHIFT, block_offset, linesize,
                                       dest[p], p);
        }
    }
#endif
