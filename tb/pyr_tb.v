//--------------------------------------------------------------------------------------------------
// Copyright (C) 2021 tianqishi
// All rights reserved
// Design    : pyr_down
// Author(s) : tianqishi
// Email     : tishi1@126.com
// QQ        : 2483210587
//-------------------------------------------------------------------------------------------------

`include "../include/lk_defines.v"

`timescale 1ns / 1ns // timescale time_unit/time_presicion

module pyr_tb;
reg rst;
reg dec_clk;

wire                                       m_axi_arready;
wire                                       m_axi_arvalid;
wire[3:0]                                  m_axi_arlen;
wire[31:0]                                 m_axi_araddr;
wire                                       m_axi_rready;
wire [63:0]                                m_axi_rdata;
wire                                       m_axi_rvalid;
wire                                       m_axi_rlast;
wire [5:0]                                 m_axi_arid;
wire [2:0]                                 m_axi_arsize;
wire [1:0]                                 m_axi_arburst;
wire [2:0]                                 m_axi_arprot;
wire [3:0]                                 m_axi_arcache;
wire [1:0]                                 m_axi_arlock;
wire [3:0]                                 m_axi_arqos;


wire                                       m_axi_awready;
wire [5:0]                                 m_axi_awid;
wire [31:0]                                m_axi_awaddr;
wire [3:0]                                 m_axi_awlen;
wire [2:0]                                 m_axi_awsize;
wire [1:0]                                 m_axi_awburst;
wire [1:0]                                 m_axi_awlock;
wire [3:0]                                 m_axi_awcache;
wire [2:0]                                 m_axi_awprot;
wire                                       m_axi_awvalid;

wire                                       m_axi_wready;
wire [5:0]                                 m_axi_wid;
wire [63:0]                                m_axi_wdata;
wire [7:0]                                 m_axi_wstrb;
wire                                       m_axi_wlast;
wire                                       m_axi_wvalid;

wire [5:0]                                 m_axi_bid;
wire [1:0]                                 m_axi_bresp;
wire                                       m_axi_bvalid;
wire                                       m_axi_bready;

wire [5:0]                                 m_axi_rrid;
wire [1:0]                                 m_axi_rresp;



initial begin

    rst <= 0;
    #100 rst <= 1;
    #100 rst <= 0;

end



always
begin
    #1 dec_clk = 0;
    #1 dec_clk = 1;
end

reg [1:0]  random_val;
always @ (posedge dec_clk)
begin
    random_val  <= $random()%4;
end

wire   [2:0]               fetch_state;
wire   [`HEIGHT_BITS-1:0]  y;
pyr pyr_inst(
    .clk                             (dec_clk),

    .rst                             (rst),

    .i_width                         (`WIDTH_BITS'd512),
    .i_height                        (`HEIGHT_BITS'd218),
    .i_stride                        (2048),
    .i_image_addr                    (32'h20000000),
    .i_pyr_image_addr                (32'h22000000),
    .i_pyr_image_stride              (2048),
    .i_border_type                   (3'd4),

    .m_axi_awready                   (m_axi_awready),
    .m_axi_awaddr                    (m_axi_awaddr),
    .m_axi_awlen                     (m_axi_awlen),
    .m_axi_awvalid                   (m_axi_awvalid),
   
    .m_axi_wready                    (m_axi_wready),
    .m_axi_wdata                     (m_axi_wdata),
    .m_axi_wstrb                     (m_axi_wstrb),
    .m_axi_wlast                     (m_axi_wlast),
    .m_axi_wvalid                    (m_axi_wvalid),


    .m_axi_arready                   (m_axi_arready),
    .m_axi_arvalid                   (m_axi_arvalid), 
    .m_axi_arlen                     (m_axi_arlen),
    .m_axi_araddr                    (m_axi_araddr),
    .m_axi_rready                    (m_axi_rready),
    .m_axi_rdata                     (m_axi_rdata),
    .m_axi_rvalid                    (m_axi_rvalid),
    .m_axi_rlast                     (m_axi_rlast),
    .o_fetch_stage                   (fetch_state),
    .y                               (y)

);

ext_ram_32 ext_ram_32
(
    .m_axi_wclk                          (dec_clk),
    .m_axi_awready                       (m_axi_awready),
    .m_axi_awid                          (m_axi_awid),
    .m_axi_awaddr                        (m_axi_awaddr),
    .m_axi_awlen                         (m_axi_awlen),
    .m_axi_awsize                        (m_axi_awsize),
    .m_axi_awburst                       (m_axi_awburst),
    .m_axi_awlock                        (m_axi_awlock),
    .m_axi_awcache                       (m_axi_awcache),
    .m_axi_awprot                        (m_axi_awprot),
    .m_axi_awvalid                       (m_axi_awvalid),
   
    .m_axi_wready                        (m_axi_wready),
    .m_axi_wid                           (m_axi_wid),
    .m_axi_wdata                         (m_axi_wdata),
    .m_axi_wstrb                         (m_axi_wstrb),
    .m_axi_wlast                         (m_axi_wlast),
    .m_axi_wvalid                        (m_axi_wvalid),
   
    .m_axi_bid                           (m_axi_bid),
    .m_axi_bresp                         (m_axi_bresp),
    .m_axi_bvalid                        (m_axi_bvalid),
    .m_axi_bready                        (m_axi_bready),

    .m_axi_clk                           (dec_clk),
    .m_axi_rst                           (rst),
    .m_axi_arready                       (m_axi_arready),
    .m_axi_arvalid                       (m_axi_arvalid),
    .m_axi_arlen                         (m_axi_arlen),
    .m_axi_araddr                        (m_axi_araddr),
    .m_axi_rready                        (m_axi_rready),
    .m_axi_rdata                         (m_axi_rdata),
    .m_axi_rvalid                        (m_axi_rvalid),
    .m_axi_rlast                         (m_axi_rlast),
    .m_axi_arid                          (m_axi_arid),
    .m_axi_arsize                        (m_axi_arsize),
    .m_axi_arburst                       (m_axi_arburst),
    .m_axi_arprot                        (m_axi_arprot),
    .m_axi_arcache                       (m_axi_arcache),
    .m_axi_arlock                        (m_axi_arlock),
    .m_axi_arqos                         (m_axi_arqos),
    .m_axi_rrid                          (m_axi_rrid),
    .m_axi_rresp                         (m_axi_rresp)
);


endmodule


