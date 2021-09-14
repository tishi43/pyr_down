`include "../include/lk_defines.v"

module pyr
(
    input wire                     clk,
    input wire                     rst,
    input wire [`WIDTH_BITS-1:0]   i_width,
    input wire [`WIDTH_BITS-1:0]   i_height,
    input wire[`STRIDE_BITS-1:0]   i_stride,
    input wire            [31:0]   i_image_addr,
    input wire[`STRIDE_BITS-1:0]   i_pyr_image_stride,
    input wire            [31:0]   i_pyr_image_addr,
    input wire            [ 2:0]   i_border_type,
    output reg            [ 2:0]   o_fetch_stage,
    output reg [`HEIGHT_BITS-1:0]  y,

    input  wire                    m_axi_arready,
    output wire                    m_axi_arvalid,
    output wire           [ 3:0]   m_axi_arlen,
    output wire           [31:0]   m_axi_araddr,

    output wire                    m_axi_rready,
    input  wire           [63:0]   m_axi_rdata,
    input  wire                    m_axi_rvalid,
    input  wire                    m_axi_rlast,

    input  wire                    m_axi_awready, // Indicates slave is ready to accept a
    output wire           [31:0]   m_axi_awaddr,  // Write address
    output wire           [ 3:0]   m_axi_awlen,   // Write Burst Length
    output reg                     m_axi_awvalid, // Write address valid

    input  wire                    m_axi_wready,  // Write data ready
    output wire           [63:0]   m_axi_wdata,    // Write data
    output reg            [ 7:0]   m_axi_wstrb,    // Write strobes
    output reg                     m_axi_wlast,    // Last write transaction
    output reg                     m_axi_wvalid    // Write valid


);

localparam
BORDER_CONSTANT    = 0, //!< `iiiiii|abcdefgh|iiiiiii`  with some specified `i`
BORDER_REPLICATE   = 1, //!< `aaaaaa|abcdefgh|hhhhhhh`
BORDER_REFLECT     = 2, //!< `fedcba|abcdefgh|hgfedcb`
BORDER_WRAP        = 3, //!< `cdefgh|abcdefgh|abcdefg`
BORDER_REFLECT_101 = 4, //!< `gfedcb|abcdefgh|gfedcba`
BORDER_TRANSPARENT = 5; //!< `uvwxyz|absdefgh|ijklmno`
//opencv pyramid不支持const，borderInterpolate不支持transparent
//wrap暂不支

// 高斯
//  1  4  6   4  1
// 4  16  24  16 4
// 6  24  36  24 6
// 4  16  24  16 4
// 1  4   6   4  1


genvar                            ii;
integer                           i;

(*mark_debug="true"*)
wire   [4:0][`WIDTH_BITS-4:0]     ram_addr_hor_conv;


wire                    [4:0]     ram_wr_ena;
wire                    [4:0]     ram_hor_conv_ena;
wire              [4:0][47:0]     ram_hor_conv_dout; //4*12=48,每项存4点,最多只存最大宽度的一半


//width=512,x1=0~64
reg          [`WIDTH_BITS-1:0]     x;
reg          [`WIDTH_BITS-3:0]     x1;
reg          [`WIDTH_BITS-1:0]     x1_d1;
reg          [`WIDTH_BITS-1:0]     x1_d2;
reg          [`WIDTH_BITS-1:0]     x1_d3;
reg          [`WIDTH_BITS-1:0]     x1_d4;
reg          [`WIDTH_BITS-1:0]     x1_d5;
reg          [`WIDTH_BITS-1:0]     x1_d6;
reg          [`WIDTH_BITS-1:0]     x1_d7;

wire         [`WIDTH_BITS-1:0]     next_x;
reg          [`WIDTH_BITS-1:0]     fetch_x;

reg         [`HEIGHT_BITS-1:0]     height_minus1;
reg          [`WIDTH_BITS-1:0]     width_minus1;
wire         [`WIDTH_BITS-1:0]     width_minus1_w;

assign width_minus1_w = i_width-1;
assign next_x = x+8;

// 以默认border_type=BORDER_REFLECT_101为例,width=512
// 水平卷积
// x= 2,1,0,1,2,
//    0,1,2,3,4
//    2,3,4,5,6    第一个8字节，得3点水平结果
//    4,5,6,7,8
//    6,7,8,9,10
//   8,9,10,11,12
//  10,11,12,13,14 第二个8字节
//  12,13,14,15,16
//  14,15,16,17,18
// 16,17,18,19,20
// 18,19,20,21,22
// 20,21,22,23,24
//...
//500,501,502,503,504
//502,503,504,505,506
//504,505,506,507,508
//506,507,508,509,510
//508,509,510,511,510

//纵向 垂直卷积
//出3行，得第一行结果，以后每出两行，得一行结果


localparam
Idle = 0,
PrepAddr = 1,
WaitOneRowDone = 2, //等待一行水平卷积结束
WaitStoreLastLine=3,
End = 4;

localparam
Delay1Cycle = 0,
AddrStage = 1,
DataStage = 2,
OutputEnd = 3;

reg     [11:0][7:0]             data_buf;

(*mark_debug="true"*)
reg    [`PIC_SIZE_BITS-1:0]     pic_offset          ; //当前要取的64字节相对图像起始地址的偏移


(*mark_debug="true"*)
reg    [`PIC_SIZE_BITS-1:0]     line_offset         ; //当前要取的行开头相对图像起始地址的偏移


(*mark_debug="true"*)
reg    [ 2:0]                   ram_write_index     ; //写入哪个ram，ram0,ram1,ram2,ram3对应0,1,2,3

reg                             write_valid         ;
reg                             write_valid_d1      ;
reg                             write_valid_d2      ;
reg                             write_valid_d3      ;
reg                             write_valid_d4      ;
reg                             write_valid_d5      ;
reg                             write_valid_d6      ;
reg                             write_valid_d7      ;
reg                             ver_conv_valid      ;
reg                             output_valid        ;
reg                    [1:0]    output_stage        ;

(*mark_debug="true"*)
reg                             last_row            ;

(*mark_debug="true"*)
reg                             first_idle          ;
(*mark_debug="true"*)
reg                             go                  ;

reg                             rst_hor_conv        ;
reg                             rst_output          ;
reg                             output_ram_sel      ;


reg                             row_first_8bytes    ; //一行开始第一比8字节
reg                             row_last_8bytes     ; //一行最后一笔8字节
wire                            row_last_burst      ; //一行最后一笔burst,一笔busrt最大8x8=64字节
reg      [2:0]                  last_arlen          ;


wire     [7:0]                  pad_first_byte      ;
wire     [7:0]                  pad_second_byte     ;

reg      [2:0]                  delay_cycles        ;

assign   m_axi_arlen        = row_last_burst?last_arlen:7;
assign   m_axi_araddr       = i_image_addr+pic_offset;
assign   m_axi_arvalid      = o_fetch_stage == PrepAddr? 1:0;
assign   m_axi_rready       = 1;

assign row_last_burst = fetch_x[`WIDTH_BITS-1:6]==width_minus1[`WIDTH_BITS-1:6];

always @ (posedge clk)
if (rst) begin
    o_fetch_stage                <= Idle;
    fetch_x                      <= 0;

    y                            <= 0;
    pic_offset                   <= 0;
    line_offset                  <= 0;
    go                           <= 1;
    first_idle                   <= 1;
    rst_hor_conv                 <= 0;
    rst_output                   <= 0;
    ver_conv_valid               <= 0;
    output_valid                 <= 0;
    ram_write_index              <= 0;
    last_row                     <= 0;
    output_ram_sel               <= 0;

    //width=513,width_minus1_w[5:3]=0,
    //width=512,width_minus1_w[5:3]=7,
    //width=511,width_minus1_w[5:3]=7,
    //width=490,width_minus1_w[5:3]=5,
    last_arlen                   <= width_minus1_w[5:3];
    width_minus1                 <= i_width-1;
    height_minus1                <= i_height-1;


end else begin
    if (o_fetch_stage == Idle && go &&output_stage==OutputEnd) begin

        if (~first_idle) begin //first_idle区分是reset之后第一次idle，还是完成一行之后回过来的idle
            pic_offset           <= line_offset+i_stride;
            line_offset          <= line_offset+i_stride;
            y                    <= y+1;
        end

        first_idle               <= 0;
        fetch_x                  <= 0;

        if (~first_idle&&y<=height_minus1) begin
            ram_write_index      <= ram_write_index==4?0:ram_write_index+1;
        end

        //y=2,4,开始3行，之后每增加两行可以算垂直卷积，这里y还未+1
        //最后一行y=奇数行，扩展一行也可以算垂直卷积
        if (y[0]||y==i_height-2) begin
            ver_conv_valid       <= 1;
        end
        else begin
            ver_conv_valid       <= 0;
        end
        if ((y&&y[0]==0)||y==height_minus1) begin
            output_valid         <= 1;
            output_ram_sel       <= ~output_ram_sel;
            rst_output           <= 1;
        end
        else begin
            output_valid         <= 0;
        end
        if (y==i_height-2)
            last_row             <= 1;

        if (y>=height_minus1) begin
            o_fetch_stage        <= WaitStoreLastLine;
        end else begin
            o_fetch_stage        <= PrepAddr;
        end

    end

    if (o_fetch_stage == PrepAddr)
        rst_output               <= 0;

    if (o_fetch_stage == PrepAddr && m_axi_arready) begin
        if (fetch_x[`WIDTH_BITS-1:6]==width_minus1[`WIDTH_BITS-1:6]) begin
            fetch_x              <= 0;
            o_fetch_stage        <= WaitOneRowDone;
        end else begin
            fetch_x              <= fetch_x+64;
            pic_offset           <= pic_offset+64;
        end
    end

    if (o_fetch_stage==WaitOneRowDone &&delay_cycles==6) begin
        o_fetch_stage            <= Idle;
    end

    if (o_fetch_stage==WaitStoreLastLine) begin
        rst_output               <= 0;
        if (output_stage==OutputEnd&&~rst_output)
            o_fetch_stage        <= End;
    end

    if (o_fetch_stage==End) begin
        ver_conv_valid           <= 0;
        output_valid             <= 0;
        rst_output               <= 0;
    end

end



assign pad_first_byte = i_border_type==BORDER_REPLICATE?m_axi_rdata[7:0]:
                        (i_border_type==BORDER_REFLECT?m_axi_rdata[15:8]:m_axi_rdata[23:16]);
assign pad_second_byte = i_border_type==BORDER_REPLICATE||i_border_type==BORDER_REFLECT?
                        m_axi_rdata[7:0]:m_axi_rdata[15:8];



reg                    conv_need_no_more_data   ;

always @ (posedge clk) begin
    if (rst) begin
        x                        <= 0;
        x1                       <= 0;
        row_first_8bytes         <= 1;
        row_last_8bytes          <= 0;
        conv_need_no_more_data   <= 0;
        write_valid              <= 0;
        delay_cycles             <= 0;
    end else if (m_axi_rvalid||conv_need_no_more_data) begin
        //行开始
        //data_buf[0][1][2]             [9][10][11]
        //         x  x  2, 1, 0,1,2,3,4,5, 6,  7,
        //第二笔8字节
        //data_buf[0][1][2]         [7][8][9][10][11]
        //         4, 5, 6,7,8,9,10,11, 12,13,14,15


        //498,499,500,501,502
        //500,501,502,503,504    width=505,不用扩展,width=504, 500,501,502,503,502
        //502,503,504,505,506                       width=506, 502,503,504,505,504
        //504,505,506,507,508                       width=508, 504,505,506,507,506
        //506,507,508,509,510                       width=510, 506,507,508,509,508 
        //508,509,510,511,510   width=512,508,509,510,511,510,不需要再取ddr了，但是还得做一次卷积，就是conv_need_no_more_data=1的情况
        //                      width=513,508,509,510,511,512,还需取一次ddr


        //data_buf    [0]    [1]   [2]   [3]   [4]   [5]   [6]    [7]    [8]    [9]    [10]   [11]
        //                                                               500,   501,   502,   503
        //data_buf    [0]    [1]   [2]   [3]   [4]   [5]   [6]    [7]    [8]    [9]    [10]   [11]
        //             500,  501,  502,  503,  504,  505,  506,   507,   508,   509,   510,   511
        //m_axi_rdata                        [7:0] [15:8][23:16][31:24][39:32][47:40][55:48][63:56]

        if (row_first_8bytes)
            data_buf             <= {m_axi_rdata,pad_second_byte,pad_first_byte,8'd0,8'd0};
        else if (conv_need_no_more_data) begin
           data_buf              <= i_border_type==BORDER_REFLECT_101?
                                       {m_axi_rdata[63:8],data_buf[10],data_buf[11:8]}:
                                       {m_axi_rdata[63:8],data_buf[11],data_buf[11:8]};
        end
        else if (row_last_8bytes) begin
            case (i_width[2:0])
                2:data_buf       <= i_border_type==BORDER_REFLECT_101?
                                   {m_axi_rdata[63:24],m_axi_rdata[7:0],m_axi_rdata[15:0],data_buf[11:8]}:
                                   {m_axi_rdata[63:24],m_axi_rdata[15:8],m_axi_rdata[15:0],data_buf[11:8]};
                4:data_buf       <= i_border_type==BORDER_REFLECT_101?
                                   {m_axi_rdata[63:40],m_axi_rdata[23:16],m_axi_rdata[31:0],data_buf[11:8]}:
                                   {m_axi_rdata[63:40],m_axi_rdata[31:24],m_axi_rdata[31:0],data_buf[11:8]};
                6:data_buf       <= i_border_type==BORDER_REFLECT_101?
                                   {m_axi_rdata[63:56],m_axi_rdata[39:32],m_axi_rdata[47:0],data_buf[11:8]}:
                                   {m_axi_rdata[63:56],m_axi_rdata[47:40],m_axi_rdata[47:0],data_buf[11:8]};
                default:data_buf <= {m_axi_rdata,data_buf[11:8]};
            endcase
        end
        else
            data_buf             <= {m_axi_rdata,data_buf[11:8]};

        if (next_x[`WIDTH_BITS-1:3]==width_minus1[`WIDTH_BITS-1:3])
            row_last_8bytes      <= 1;
        else
            row_last_8bytes      <= 0;

        if (x[`WIDTH_BITS-1:3]==width_minus1[`WIDTH_BITS-1:3]) begin
            if (i_width[2:0]==0) begin
                conv_need_no_more_data  <= 1;
                x                       <= x+8;
                x1                      <= x1+1;
            end
            else begin
                x                       <= 0;
                x1                      <= 0;
                delay_cycles            <= 1;
                row_first_8bytes        <= 1;
            end

        end
        else if (conv_need_no_more_data) begin
            x                           <= 0;
            x1                          <= 0;
            row_first_8bytes            <= 1;
            conv_need_no_more_data      <= 0;
            delay_cycles                <= 1;
        end
        else begin
            x                           <= x+8;
            x1                          <= x1+1;
            row_first_8bytes            <= 0;
        end
        write_valid                     <= 1;
    end
    else begin
        write_valid                     <= 0;
        if (delay_cycles && delay_cycles<=6)
            delay_cycles                <= delay_cycles+1;
    end

end

always @ (posedge clk) begin
    x1_d1           <= x1;
    x1_d2           <= x1_d1;
    x1_d3           <= x1_d2;
    x1_d4           <= x1_d3;
    x1_d5           <= x1_d4;
    x1_d6           <= x1_d5;
    x1_d7           <= x1_d6;

    write_valid_d1  <= write_valid;
    write_valid_d2  <= write_valid_d1;
    write_valid_d3  <= write_valid_d2;
    write_valid_d4  <= write_valid_d3;
    write_valid_d5  <= write_valid_d4;
    write_valid_d6  <= write_valid_d5;
    write_valid_d7  <= write_valid_d6;
end

//1+4+6+4+1=16
reg       [3:0][11:0]    hor_conv_result;
reg       [3:0][11:0]    hor_conv_temp1; //1+4+4=9
reg       [3:0][10:0]    hor_conv_temp2; //2+4+1=7

// [0]  0,1,2,3,4
// [1]  2,3,4,5,6
// [2]  4,5,6,7,8
// [3]  6,7,8,9,10
always @ (posedge clk) begin
    for (i=0;i<4;i=i+1) begin
        hor_conv_temp1[i]     <= data_buf[i*2]+{data_buf[i*2+1],2'b00}+{data_buf[i*2+2],2'b00};
        hor_conv_temp2[i]     <= {data_buf[i*2+2],1'b0}+{data_buf[i*2+3],2'b00}+data_buf[i*2+4];

        hor_conv_result[i]    <= hor_conv_temp1[i]+hor_conv_temp2[i];
    end
end

generate
    for (ii=0;ii<5;ii=ii+1)
    begin: ram_addr_label
        assign ram_addr_hor_conv[ii] = ram_write_index==ii? x1_d3:x1_d2;
    end
endgenerate

generate
    for (ii=0;ii<5;ii=ii+1)
    begin: ram_wr_ena_label
        assign ram_wr_ena[ii] =  ram_write_index==ii&&write_valid_d2;
        assign ram_hor_conv_ena[ii] = ram_write_index==ii||ver_conv_valid;
    end
endgenerate


generate
    for (ii=0;ii<5;ii=ii+1)
    begin: ram_hor_conv_label
        ram #(`WIDTH_BITS-3, 48) ram_hor_conv(
            .clk(clk),
            .en(ram_hor_conv_ena[ii]),
            .we(ram_wr_ena[ii]),
            .addr(ram_addr_hor_conv[ii]),
            .data_in(hor_conv_result),
            .data_out(ram_hor_conv_dout[ii])
        );


    end
endgenerate

wire       [2:0]     data_index0;
wire       [2:0]     data_index1;
wire       [2:0]     data_index2;
wire       [2:0]     data_index3;

reg  [3:0][11:0]     ver_conv_data0;
reg  [3:0][11:0]     ver_conv_data1;
reg  [3:0][11:0]     ver_conv_data2;
reg  [3:0][11:0]     ver_conv_data3;
reg  [3:0][11:0]     ver_conv_data4;

assign data_index0 = ram_write_index>=4?ram_write_index-4:ram_write_index+1;
assign data_index1 = ram_write_index>=3?ram_write_index-3:ram_write_index+2;
assign data_index2 = ram_write_index>=2?ram_write_index-2:ram_write_index+3;
assign data_index3 = ram_write_index>=1?ram_write_index-1:4;

always @(posedge clk) begin
    if (y==2) begin
        ver_conv_data0       <= i_border_type==BORDER_REPLICATE?ram_hor_conv_dout[0]:
                                (i_border_type==BORDER_REFLECT?ram_hor_conv_dout[1]:hor_conv_result);
        ver_conv_data1       <= i_border_type==BORDER_REPLICATE||i_border_type==BORDER_REFLECT?
                                 ram_hor_conv_dout[0]:ram_hor_conv_dout[1];
        ver_conv_data2       <= ram_hor_conv_dout[0];
        ver_conv_data3       <= ram_hor_conv_dout[1];
        ver_conv_data4       <= hor_conv_result;
    end
    else if (last_row && y[0]) begin
        ver_conv_data0       <= ram_hor_conv_dout[data_index1];
        ver_conv_data1       <= ram_hor_conv_dout[data_index2];
        ver_conv_data2       <= ram_hor_conv_dout[data_index3];
        ver_conv_data3       <= hor_conv_result;
        ver_conv_data4       <= i_border_type==BORDER_REPLICATE||i_border_type==BORDER_REFLECT?
                                hor_conv_result:ram_hor_conv_dout[data_index3];
    end
    else begin
        ver_conv_data0       <= ram_hor_conv_dout[data_index0];
        ver_conv_data1       <= ram_hor_conv_dout[data_index1];
        ver_conv_data2       <= ram_hor_conv_dout[data_index2];
        ver_conv_data3       <= ram_hor_conv_dout[data_index3];
        ver_conv_data4       <= hor_conv_result;
    end

end

//1+4+6+4+1=16
reg       [3:0][15:0]    ver_conv_result;
reg       [3:0][15:0]    ver_conv_temp1; //1+4+4=9
reg       [3:0][14:0]    ver_conv_temp2; //2+4+1=7


always @ (posedge clk) begin
    for (i=0;i<4;i=i+1) begin
        ver_conv_temp1[i]     <= ver_conv_data0[i]+{ver_conv_data1[i],2'b00}+{ver_conv_data2[i],2'b00};
        ver_conv_temp2[i]     <= {ver_conv_data2[i],1'b0}+{ver_conv_data3[i],2'b00}+ver_conv_data4[i];

        ver_conv_result[i]    <= ver_conv_temp1[i]+ver_conv_temp2[i]+128;
    end
end


reg            [`WIDTH_BITS-4:0]    x_output             ; //每个x存4点
reg            [`WIDTH_BITS-4:0]    x_output_max         ;
wire           [`WIDTH_BITS-2:0]    output_width         ;
reg       [1:0][`WIDTH_BITS-4:0]    output_ram_addr      ;
reg       [1:0]                     output_ram_wr_ena    ;
wire                 [1:0][31:0]    output_ram_dout      ;
reg                       [87:0]    write_buffer         ; //11bytes,第一笔3字节有效
reg                                 write_buffer_ready   ;
reg         [`PIC_SIZE_BITS-1:0]    output_offset        ;
reg         [`PIC_SIZE_BITS-1:0]    output_line_offset   ;
reg                                 first_line           ;
reg                                 last_output          ; //一行最后一笔ddr输出
reg                        [7:0]    last_strb            ;
wire            [`WIDTH_BITS-3:0]   output_width_plus4   ;

assign output_width = i_width[`WIDTH_BITS-1:1];
assign output_width_plus4 = output_width+4;

generate
    for (ii=0;ii<2;ii=ii+1)
    begin: output_ram_label
        ram #(`WIDTH_BITS-3,32) output_ram_inst(
            .clk(clk),
            .en(output_valid||ver_conv_valid),
            .we(output_ram_wr_ena[ii]),
            .addr(output_ram_addr[ii]),
            .data_in({ver_conv_result[3][15:8],
                     ver_conv_result[2][15:8],
                     ver_conv_result[1][15:8],
                     ver_conv_result[0][15:8]}),
            .data_out(output_ram_dout[ii])
        );
    end
endgenerate

always @ (*) begin

    if (output_ram_sel) begin
        output_ram_addr[0]       = x_output;
        output_ram_addr[1]       = x1_d6;
        output_ram_wr_ena[0]     = 0;
        output_ram_wr_ena[1]     = write_valid_d5;
    end
    else begin
        output_ram_addr[0]       = x1_d6;
        output_ram_addr[1]       = x_output;
        output_ram_wr_ena[0]     = write_valid_d5;
        output_ram_wr_ena[1]     = 0;
    end
end

//width=512,513,输出256，
always @ (posedge clk) begin
    case (i_width[3:1])
        0:    last_strb  <= 8'b11111111;
        1:    last_strb  <= 8'b00000001;
        2:    last_strb  <= 8'b00000011;
        3:    last_strb  <= 8'b00000111;
        4:    last_strb  <= 8'b00001111;
        5:    last_strb  <= 8'b00011111;
        6:    last_strb  <= 8'b00111111;
        7:    last_strb  <= 8'b01111111;
    endcase
end

assign m_axi_awaddr = i_pyr_image_addr+output_offset;
assign m_axi_wdata = write_buffer[63:0];
assign m_axi_awlen = 0;

always @ (posedge clk)
    x_output_max    <= {output_width_plus4[`WIDTH_BITS-3:3],1'b0};

always @ (posedge clk) begin
    if (rst) begin
        m_axi_awvalid               <= 0;
        m_axi_wlast                 <= 0;
        m_axi_wvalid                <= 0;
        write_buffer_ready          <= 0;
        output_line_offset          <= 0;
        output_offset               <= 0;
        last_output                 <= 0;
        first_line                  <= 1;
        output_stage                <= OutputEnd;
    end
    else if (rst_output) begin
        x_output                    <= 0;
        output_stage                <= Delay1Cycle;
        write_buffer_ready          <= 0;
        if (~first_line) begin
            output_line_offset      <= output_line_offset+i_pyr_image_stride;
            output_offset           <= output_line_offset+i_pyr_image_stride;
        end
        first_line                  <= 0;
        last_output                 <= 0;
    end
    else begin
        if (output_stage==Delay1Cycle) begin
            output_stage            <= AddrStage;
            x_output                <= x_output+1;
            m_axi_awvalid           <= 1;
        end

        if (output_stage==AddrStage) begin
            if (~write_buffer_ready) begin
                //每行第一次，要移入3个4字节，之后，每次移入2个4字节，第一个4字节在下面DataStage移入，这里移入第二个4字节
                x_output            <= x_output+1;
                //width=512,输出256点
                //                                                  x_output=64(下面DataStage)   x_output=65(这里)
                //{x,0,1,2} {3,4,5,6} {7,8,9,10} ... {247,248,249,250}     {251,252,253,254}      {255,x,x,x}

                //width=511,输出255点                                                            最后也要推一笔，虽然是xxxx，需凑够8字节
                //{x,0,1,2} {3,4,5,6} {7,8,9,10} ... {247,248,249,250}     {251,252,253,254}   {x,x,x,x}
                //databuf       248,249,250,  251,252,253,254,   x,x,x,x
                //m_axi_rdata   248,249,250,  251,252,253,254,   x

                //width=502,输出251点, 取到x_output=62
                //251-3=248   1111 1000 + 7 =   1111 1111
                //256-3=253   1111 1101 +7 =  1 0000 0100
                if (x_output>=x_output_max)
                    last_output     <= 1;
                write_buffer        <= output_ram_sel?{output_ram_dout[0],write_buffer[87:32]}:
                                                        {output_ram_dout[1],write_buffer[87:32]};
            end
            if (m_axi_awready) begin
                m_axi_awvalid       <= 0;
            end
            if (x_output>=3&&x_output[0]) begin
                write_buffer_ready  <= 1;
                if (m_axi_awready||m_axi_awvalid==0) begin
                    output_stage    <= DataStage;
                    m_axi_wvalid    <= 1;
                    m_axi_wlast     <= 1;
                    m_axi_wstrb     <= x_output>=x_output_max?last_strb:8'b1111_1111;
                    output_offset   <= output_offset+8;
                end
            end
        end

        if (output_stage==DataStage&&m_axi_wready) begin
            m_axi_wvalid            <= 0;
            m_axi_wlast             <= 0;
            write_buffer_ready      <= 0;
            if (last_output) begin
                output_stage        <= OutputEnd;
            end
            else begin
                output_stage        <= AddrStage;
                m_axi_awvalid       <= 1;
                x_output            <= x_output+1;
                write_buffer        <= output_ram_sel?{output_ram_dout[0],write_buffer[87:32]}:
                                                        {output_ram_dout[1],write_buffer[87:32]};
            end
        end
    end
end



`ifdef RANDOM_INIT
integer  seed;
integer random_val;
initial  begin
    seed                               = $get_initial_random_seed(); 
    random_val                         = $random(seed);

end
`endif



endmodule

