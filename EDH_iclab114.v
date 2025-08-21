// `include "../00_TESTBED/MEM_MAP_define.v"
// synopsys translate_off
`include "/usr/synthesis/dw/sim_ver/DW_minmax.v"
// synopsys translate_on


module EDH #(parameter ID_WIDTH=4, DATA_WIDTH=128, ADDR_WIDTH=32) (
    clk,	
	rst_n,	
    in_valid,
    pic_no,
    se_no,
    op,
    busy,

    awid_m_inf,
    awaddr_m_inf,
    awsize_m_inf,
    awburst_m_inf,
    awlen_m_inf,
    awvalid_m_inf,
    awready_m_inf,
                
    wdata_m_inf,
    wlast_m_inf,
    wvalid_m_inf,
    wready_m_inf,
                
    bid_m_inf,
    bresp_m_inf,
    bvalid_m_inf,
    bready_m_inf,
                
    arid_m_inf,
    araddr_m_inf,
    arlen_m_inf,
    arsize_m_inf,
    arburst_m_inf,
    arvalid_m_inf,
                
    arready_m_inf, 
    rid_m_inf,
    rdata_m_inf,
    rresp_m_inf,
    rlast_m_inf,
    rvalid_m_inf,
    rready_m_inf 
);

input in_valid, clk, rst_n;
input [3:0] pic_no;
input [5:0] se_no;
input [1:0] op;

output reg busy;

// axi write address channel 
output  [ID_WIDTH-1:0]     awid_m_inf;
output reg [ADDR_WIDTH-1:0] awaddr_m_inf;
output  [2:0]            awsize_m_inf;
output  [1:0]           awburst_m_inf;
output reg [7:0]             awlen_m_inf;
output reg                awvalid_m_inf;
input                 awready_m_inf;
// axi write data channel 
output [DATA_WIDTH-1:0]  wdata_m_inf;
output reg                  wlast_m_inf;
output reg                wvalid_m_inf;
input                  wready_m_inf;
// axi write response channel
input [ID_WIDTH-1:0]      bid_m_inf;
input [1:0]             bresp_m_inf;
input                  bvalid_m_inf;
output reg                  bready_m_inf;
// -----------------------------
// axi read address channel 
output    [ID_WIDTH-1:0]      arid_m_inf;
output reg   [ADDR_WIDTH-1:0]  araddr_m_inf;
output reg   [7:0]              arlen_m_inf;
output    [2:0]             arsize_m_inf;
output    [1:0]            arburst_m_inf;
output reg                 arvalid_m_inf;
input                     arready_m_inf;
// -----------------------------
// axi read data channel 
input [ID_WIDTH-1:0]          rid_m_inf;
input [DATA_WIDTH-1:0]      rdata_m_inf;
input [1:0]                 rresp_m_inf;
input                       rlast_m_inf;
input                     rvalid_m_inf;
output  reg                    rready_m_inf;

reg out_ce, out_oe, out_we;
// reg [5:0] se_a;
reg [7:0] out_a, t_out_a;
// reg [127:0] se_d, out_d;
reg [127:0] pic_arr [0:3][0:3];
reg [127:0] se_arr;
reg [8:0] t_erosion [0:15][0:15];
reg [7:0] erosion [0:15][0:15];
wire [7:0] pic_padd [0:3][0:2];
reg padding;
wire min_max;
wire [3:0] idx_ip;
wire [7:0] min_erosion [0:15];

wire [31:0] pic_baddr, se_baddr;

reg [2:0] counter;
reg [7:0] counter_2;

reg con_count, first_valid, n_pic;

reg [5:0] t_se_no;
reg [3:0] t_pic_no;

reg [12:0] cdf_c1 [0:255];
reg [12:0] t_cdf_c1 [0:255][0:15];
wire [12:0] cdf_min, cdf_max;
reg [7:0] cdf_out [0:15];
reg [20:0] temp_cdf [0:15];
reg [20:0] n_temp_cdf [0:15];
reg [7:0] n_temp_cdf2 [0:15];
reg [7:0] store1 [0:15];
reg [7:0] store2 [0:15];
// reg [7:0] max_idx, min_idx;
reg [7:0] min_idx;
wire [127:0] ero_his;
wire min_max_his;
// reg [127:0] se_arr;
wire [127:0] out_d, out_q;
reg [12:0] his_min;
integer i, j;
parameter IDLE = 3'd0,
          SE = 3'd1,
          PIC = 3'd2,
          OUT = 3'd3,
          HIST = 3'd4;

parameter burst = 2'b01;
parameter pic_len = 8'd255;
parameter se_len = 8'd0;
parameter size = 3'b100;
parameter id = 4'd0;
parameter resp = 2'b00;

reg [2:0] n_state, c_state;
reg [1:0] t_op;

// RA1SH_SE S1(.CLK(clk), .A(se_a), .D(se_d), .CEN(se_ce), .OEN(se_oe), .WEN(se_we), .Q(se_q));
RA1SH_PIC O1(.CLK(clk), .A(out_a), .D(out_d), .CEN(out_ce), .OEN(out_oe), .WEN(out_we), .Q(out_q));
assign out_d = (t_op == 2'd2) ? rdata_m_inf : {min_erosion[15], min_erosion[14], min_erosion[13], min_erosion[12], min_erosion[11], min_erosion[10], min_erosion[9], min_erosion[8]
                , min_erosion[7], min_erosion[6], min_erosion[5], min_erosion[4], min_erosion[3], min_erosion[2], min_erosion[1], min_erosion[0]};
assign pic_baddr = 32'h40000;
assign se_baddr = 32'h30000;
assign wdata_m_inf = (t_op == 2'd2) ? {cdf_out[15], cdf_out[14], cdf_out[13], cdf_out[12], cdf_out[11], cdf_out[10], cdf_out[9], cdf_out[8]
                                        , cdf_out[7], cdf_out[6], cdf_out[5], cdf_out[4], cdf_out[3], cdf_out[2], cdf_out[1], cdf_out[0]} : out_q;
assign min_max = (t_op == 2'd0) ? 1'b0 : 1'b1;
// assign idx_ip = (t_op == 2'd0) ? 4'd0 : 4'd15;

assign awburst_m_inf = burst;
assign awsize_m_inf = size;
assign awid_m_inf = id;

assign arid_m_inf = id;
assign arsize_m_inf = size;
assign arburst_m_inf = burst;

assign min_max_his = (t_op == 2'd2) ? 1'b0 : min_max;
// assign min_max_his[1] = (t_op == 2'd2) ? 1'b1 : min_max;

assign cdf_min = cdf_c1[min_idx];
// assign his_min = 13'd4096 -cdf_min; 
// assign cdf_max = cdf_c2[max_idx];
genvar m, n;
generate
   
    for(n = 0; n < 16; n = n + 1) begin
        for(m = 0; m < 4; m = m + 1)begin
            // for(l = 0; l < 4; l = l + 1) begin
            //     always@(*) begin
            //     if(n >= 13)
            //         erosion[16*n+4*m+l] = pic_arr[m][0][127-8*n-8*l:120-8*n-8*l] - se_arr[127-32*m:120-32*m];
            //     end
            // end
            if(n == 13) begin
                always@(*) begin
                        t_erosion[n][4*m] = (t_op == 2'd0) ? {1'b0,pic_arr[m][0][8*n+:8]} - {1'b0,se_arr[32*m+7:32*m]} : {1'b0,pic_arr[m][0][8*n+:8]} + {1'b0,se_arr[127-32*m:120-32*m]};
                        t_erosion[n][4*m+1] = (t_op == 2'd0) ? {1'b0,pic_arr[m][0][8*n+8+:8]} - {1'b0,se_arr[32*m+15:32*m+8]} : {1'b0,pic_arr[m][0][8*n+8+:8]} + {1'b0,se_arr[119-32*m:112-32*m]};
                        t_erosion[n][4*m+2] = (t_op == 2'd0) ? {1'b0,pic_arr[m][0][8*n+16+:8]} - {1'b0,se_arr[32*m+23:32*m+16]} : {1'b0,pic_arr[m][0][8*n+16+:8]} + {1'b0,se_arr[111-32*m:104-32*m]};
                        t_erosion[n][4*m+3] = (t_op == 2'd0) ? {1'b0,pic_padd[m][0]} - {1'b0,se_arr[32*m+31:32*m+24]} : {1'b0,pic_padd[m][0]} + {1'b0,se_arr[103-32*m:96-32*m]};
                end
            end        
                   
            else if(n == 14) begin
                always@(*) begin
                    t_erosion[n][4*m] = (t_op == 2'd0) ? {1'b0,pic_arr[m][0][8*n+:8]} - {1'b0,se_arr[32*m+7:32*m]} : {1'b0,pic_arr[m][0][8*n+:8]} + {1'b0,se_arr[127-32*m:120-32*m]};
                    t_erosion[n][4*m+1] = (t_op == 2'd0) ? {1'b0,pic_arr[m][0][8*n+8+:8]} - {1'b0,se_arr[32*m+15:32*m+8]} : {1'b0,pic_arr[m][0][8*n+8+:8]} + {1'b0,se_arr[119-32*m:112-32*m]};
                    t_erosion[n][4*m+2] = (t_op == 2'd0) ? {1'b0,pic_padd[m][0]} - {1'b0,se_arr[32*m+23:32*m+16]} : {1'b0,pic_padd[m][0]} + {1'b0,se_arr[111-32*m:104-32*m]};
                    t_erosion[n][4*m+3] = (t_op == 2'd0) ? {1'b0,pic_padd[m][1]} - {1'b0,se_arr[32*m+31:32*m+24]} : {1'b0,pic_padd[m][1]} + {1'b0,se_arr[103-32*m:96-32*m]};
                end
                   
            end
            else if(n == 15) begin
                    // t_erosion[n][4*m] = (t_op == 2'd0) ? pic_arr[m][0][127-8*n:120-8*n] - se_q[127-32*m:120-32*m] : pic_arr[m][0][127-8*n:120-8*n] + se_q[127-32*m:120-32*m];
                    // t_erosion[n][4*m+1] = (t_op == 2'd0) ? pic_padd[m][0] - se_q[119-32*m:112-32*m] : pic_padd[m][0] + se_q[119-32*m:112-32*m];
                    // t_erosion[n][4*m+2] = (t_op == 2'd0) ? pic_padd[m][1] - se_q[111-32*m:104-32*m] : pic_padd[m][1] + se_q[111-32*m:104-32*m];
                    // t_erosion[n][4*m+3] = (t_op == 2'd0) ? pic_padd[m][2] - se_q[103-32*m:96-32*m] : pic_padd[m][2] + se_q[103-32*m:96-32*m];
                    always@(*) begin
                        t_erosion[n][4*m] = (t_op == 2'd0) ? {1'b0,pic_arr[m][0][8*n+:8]} - {1'b0,se_arr[32*m+7:32*m]} : {1'b0,pic_arr[m][0][8*n+:8]} + {1'b0,se_arr[127-32*m:120-32*m]};
                        t_erosion[n][4*m+1] = (t_op == 2'd0) ? {1'b0,pic_padd[m][0]} - {1'b0,se_arr[32*m+15:32*m+8]} : {1'b0,pic_padd[m][0]} + {1'b0,se_arr[119-32*m:112-32*m]};
                        t_erosion[n][4*m+2] = (t_op == 2'd0) ? {1'b0,pic_padd[m][1]} - {1'b0,se_arr[32*m+23:32*m+16]} : {1'b0,pic_padd[m][1]} + {1'b0,se_arr[111-32*m:104-32*m]};
                        t_erosion[n][4*m+3] = (t_op == 2'd0) ? {1'b0,pic_padd[m][2]} - {1'b0,se_arr[32*m+31:32*m+24]} : {1'b0,pic_padd[m][2]} + {1'b0,se_arr[103-32*m:96-32*m]};
                    end
                   
            end
                
            else begin
                always@(*) begin
                    t_erosion[n][4*m] = (t_op == 2'd0) ? {1'b0,pic_arr[m][0][8*n+:8]} - {1'b0,se_arr[32*m+7:32*m]} : {1'b0,pic_arr[m][0][8*n+:8]} + {1'b0,se_arr[127-32*m:120-32*m]};
                    t_erosion[n][4*m+1] = (t_op == 2'd0) ? {1'b0,pic_arr[m][0][8*n+8+:8]} - {1'b0,se_arr[32*m+15:32*m+8]} : pic_arr[m][0][8*n+8+:8] + {1'b0,se_arr[119-32*m:112-32*m]};
                    t_erosion[n][4*m+2] = (t_op == 2'd0) ? {1'b0,pic_arr[m][0][8*n+16+:8]} - {1'b0,se_arr[32*m+23:32*m+16]} : {1'b0,pic_arr[m][0][8*n+16+:8]} + {1'b0,se_arr[111-32*m:104-32*m]};
                    t_erosion[n][4*m+3] = (t_op == 2'd0) ? {1'b0,pic_arr[m][0][8*n+24+:8]} - {1'b0,se_arr[32*m+31:32*m+24]} : {1'b0,pic_arr[m][0][8*n+24+:8]} + {1'b0,se_arr[103-32*m:96-32*m]};


                end
                   
                
            end
            always@(*) begin
                if(t_erosion[n][4*m][8]) begin
                    if(t_op == 2'd0) begin
                        erosion[n][4*m] = 8'd0;
                    end
                    else begin
                        erosion[n][4*m] = 8'd255;
                        
                    end
                end
            
                else begin
                    
                    erosion[n][4*m] = t_erosion[n][4*m];
                
                end
            end

            always@(*) begin
                if(t_erosion[n][4*m+1][8]) begin
                    if(t_op == 2'd0) begin
                        
                        erosion[n][4*m+1] = 8'd0;
                        
                    end
                    else begin
                        
                        erosion[n][4*m+1] = 8'd255;
                    
                    end
                end
                else begin
                    
                    erosion[n][4*m+1] = t_erosion[n][4*m+1];
                
                end
            end

            always@(*) begin
                if(t_erosion[n][4*m+2][8]) begin
                    if(t_op == 2'd0) begin
                        
                        erosion[n][4*m+2] = 8'd0;
                        
                    end
                    else begin
                        
                        erosion[n][4*m+2] = 8'd255;
                        
                    end
                end
                else begin
                    
                    erosion[n][4*m+2] = t_erosion[n][4*m+2];
                

                end

            end
            always@(*) begin
                if(t_erosion[n][4*m+3][8]) begin
                    if(t_op == 2'd0) begin
                        
                        erosion[n][4*m+3] = 8'd0;
                        
                    end
                    else begin
                       
                        erosion[n][4*m+3] = 8'd255;
                        
                    end
                end
                else begin
                   
                    erosion[n][4*m+3] = t_erosion[n][4*m+3];
                    
                end
            end
      
        end
        
    end
    for(m = 0; m < 4; m = m + 1) begin
        for(n = 0; n < 3; n = n + 1) begin
            // assign pic_padd[m][n] = (padding) ? 8'd0 : pic_arr[m][1][127-8*n:120-8*n];
            assign pic_padd[m][n] = (padding) ? 8'd0 : pic_arr[m][1][8*n+7:8*n];
        end
    end
    for(m = 0; m < 16; m = m + 1) begin
        if(m == 0) begin
            DW_minmax #(8, 16) Um (.a(ero_his), .tc(1'b0), .min_max(min_max_his), .value(min_erosion[m]), .index(idx_ip));
        end
      

        else begin
            DW_minmax #(8, 16) Um (.a({erosion[m][0],erosion[m][1],erosion[m][2],erosion[m][3],erosion[m][4],erosion[m][5],erosion[m][6],erosion[m][7],erosion[m][8],erosion[m][9],erosion[m][10],erosion[m][11],erosion[m][12],erosion[m][13],erosion[m][14],erosion[m][15]}), .tc(1'b0), .min_max(min_max),
            .value(min_erosion[m]), .index(idx_ip));
        end
        
    end
  
    
    // for(m = 0; m < 256; m = m + 1) begin
    //     for(n = 0; n < 16; n = n + 1) begin

    //         if(n == 0) begin
    //             always@(*) begin
    //                 if(rdata_m_inf[8*n+:8] <= m) begin
    //                     t_cdf_c1[m][n] = cdf_c1[m] + 5'd1;
    //                 end
    //                 else begin
    //                     t_cdf_c1[m][n] = cdf_c1[m];
    //                 end

    //             end

    //         end
    //         else begin
    //             always@(*) begin
    //                 if(rdata_m_inf[8*n+:8] <= m) begin
    //                     t_cdf_c1[m][n] = t_cdf_c1[m][n-1] + 5'd1;
    //                 end
    //                 else begin
    //                     t_cdf_c1[m][n] = t_cdf_c1[m][n-1];
    //                 end

    //             end
    //         end

    //     end
        
    // end
   
    
    for(m = 0; m < 1; m = m + 1) begin
        assign ero_his = (t_op == 2'd2) ? rdata_m_inf : {erosion[m][0],erosion[m][1],erosion[m][2],erosion[m][3],erosion[m][4],erosion[m][5],erosion[m][6],erosion[m][7],erosion[m][8],erosion[m][9],erosion[m][10],erosion[m][11],erosion[m][12],erosion[m][13],erosion[m][14],erosion[m][15]};

    end
   
    
endgenerate

always@(*) begin
    for(i = 0; i < 256; i = i + 1) begin
        for(j = 0; j < 16; j = j + 1) begin

            if(j == 0) begin
               
                if(rdata_m_inf[8*j+:8] <= i) begin
                    t_cdf_c1[i][j] = cdf_c1[i] + 5'd1;
                end
                else begin
                    t_cdf_c1[i][j] = cdf_c1[i];
                end

               

            end
            else begin
                
                if(rdata_m_inf[8*j+:8] <= i) begin
                    t_cdf_c1[i][j] = t_cdf_c1[i][j-1] + 5'd1;
                end
                else begin
                    t_cdf_c1[i][j] = t_cdf_c1[i][j-1];
                end

                
            end

        end
        
    end
   

end
always@(*) begin
    temp_cdf[0] = (cdf_c1[out_q[7-:8]] - cdf_min)*8'd255;
    temp_cdf[1] = (cdf_c1[out_q[15-:8]] - cdf_min)*8'd255;
    temp_cdf[2] = (cdf_c1[out_q[23-:8]] - cdf_min)*8'd255;
    temp_cdf[3] = (cdf_c1[out_q[31-:8]] - cdf_min)*8'd255;
    temp_cdf[4] = (cdf_c1[out_q[39-:8]] - cdf_min)*8'd255;
    temp_cdf[5] = (cdf_c1[out_q[47-:8]] - cdf_min)*8'd255;
    temp_cdf[6] = (cdf_c1[out_q[55-:8]] - cdf_min)*8'd255;
    temp_cdf[7] = (cdf_c1[out_q[63-:8]] - cdf_min)*8'd255;
    temp_cdf[8] = (cdf_c1[out_q[71-:8]] - cdf_min)*8'd255;
    temp_cdf[9] = (cdf_c1[out_q[79-:8]] - cdf_min)*8'd255;
    temp_cdf[10] = (cdf_c1[out_q[87-:8]] - cdf_min)*8'd255;
    temp_cdf[11] = (cdf_c1[out_q[95-:8]] - cdf_min)*8'd255;
    temp_cdf[12] = (cdf_c1[out_q[103-:8]] - cdf_min)*8'd255;
    temp_cdf[13] = (cdf_c1[out_q[111-:8]] - cdf_min)*8'd255;
    temp_cdf[14] = (cdf_c1[out_q[119-:8]] - cdf_min)*8'd255;
    temp_cdf[15] = (cdf_c1[out_q[127-:8]] - cdf_min)*8'd255;
    n_temp_cdf2[0] = n_temp_cdf[0] / his_min;
    n_temp_cdf2[1] = n_temp_cdf[1] / his_min;
    n_temp_cdf2[2] = n_temp_cdf[2] / his_min;
    n_temp_cdf2[3] = n_temp_cdf[3] / his_min;
    n_temp_cdf2[4] = n_temp_cdf[4] / his_min;
    n_temp_cdf2[5] = n_temp_cdf[5] / his_min;
    n_temp_cdf2[6] = n_temp_cdf[6] / his_min;
    n_temp_cdf2[7] = n_temp_cdf[7] / his_min;
    n_temp_cdf2[8] = n_temp_cdf[8] / his_min;
    n_temp_cdf2[9] = n_temp_cdf[9] / his_min;
    n_temp_cdf2[10] = n_temp_cdf[10] / his_min;
    n_temp_cdf2[11] = n_temp_cdf[11] / his_min;
    n_temp_cdf2[12] = n_temp_cdf[12] / his_min;
    n_temp_cdf2[13] = n_temp_cdf[13] / his_min;
    n_temp_cdf2[14] = n_temp_cdf[14] / his_min;
    n_temp_cdf2[15] = n_temp_cdf[15] / his_min;

end
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        c_state <= IDLE;
    end

    else begin
        c_state <= n_state;

    end
end


always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        busy <= 1'b0;
    end

    else if(n_state == IDLE) begin
        busy <= 1'b0;  
    end

    else if(!in_valid)begin
        busy <= 1'b1;
    end
end
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        padding <= 1'b0;
    end

    else if(n_state == IDLE) begin
        padding <= 1'b0;  
    end

    else if(n_state == PIC) begin
        if(counter == 2'd2 && counter_2 >= 8'd4) begin
            padding <= 1'b1;
        end
        else begin
            padding <= 1'b0;
        end
    end
end
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        counter <= 3'd0;
    end
    else if(n_state == IDLE) begin
        counter <= 3'd0;  
    end

    else if(n_state == PIC) begin


        if(con_count || (rready_m_inf && rvalid_m_inf)) begin
            if(counter == 3'd3) begin
                counter <= 3'd0;
            end
            else begin
                counter <= counter + 3'd1;
            end
        end

    end
    else if(n_state == OUT) begin
        if(counter != 3'd4) begin
            counter <= counter + 3'd1;

        end
    end
    // else if(n_state == HIST) begin
    //     if(counter == 2'd1) begin


    //     end

    // end
end
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        con_count <= 1'b0;
    end
    else if(n_state == IDLE) begin
        con_count <= 1'b0;  
    end

    else if(n_state == PIC || n_state == HIST) begin
        if(rready_m_inf && rvalid_m_inf) begin
            con_count <= 1'b1;
          
        end

    end
    else if(n_state == OUT && t_op == 2'd2) begin
        con_count <= 1'b0;
    end
    // else begin
    //     con_count <= 1'b0;
    // end
end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        counter_2 <= 8'd0;
    end
    else if(n_state == IDLE) begin
        counter_2 <= 8'd0;  
    end

    else if(n_state == PIC) begin
        if(con_count || (rready_m_inf && rvalid_m_inf)) begin
            
            if(counter == 3'd3) begin
                if(counter_2 == 8'd67) begin
                    counter_2 <= 8'd0;
                end
                else begin
                    counter_2 <= counter_2 + 8'd1;
                end
            end

          
        end

    end
    else if(n_state == HIST) begin
        if(rready_m_inf && rvalid_m_inf) begin
            
            
            if(counter_2 == 8'd255) begin
                counter_2 <= 8'd0;
            end
            else begin
                counter_2 <= counter_2 + 8'd1;
            end
           

          
        end

    end
    else if(n_state == OUT) begin
        if(wvalid_m_inf && wready_m_inf) begin
           
            counter_2 <= counter_2 + 8'd1;
            
        end

    end
end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        first_valid <= 1'b1;
    end
    else if(n_state == IDLE) begin
        first_valid <= 1'b1;
    end
    else if(n_state == PIC) begin
        if(first_valid) begin
            first_valid <= 1'b0;
        end

    end
   

end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        arvalid_m_inf <= 1'b0;
    end
    else if(n_state == IDLE) begin
        arvalid_m_inf <= 1'b0;
    end
    else if(n_state == SE || n_state == HIST) begin
        //
        if(arready_m_inf && arvalid_m_inf) begin//
            arvalid_m_inf <= 1'b0;
        end
        // else if(first_valid) begin
        //     arvalid_m_inf <= 1'b1;
        // end
        else if(in_valid) begin
            arvalid_m_inf <= 1'b1;
        end

    end
    else if(n_state == PIC) begin
        if(arready_m_inf && arvalid_m_inf) begin
            arvalid_m_inf <= 1'b0;
        end
        // else if(first_valid) begin
        //     arvalid_m_inf <= 1'b1;
        // end
        else if(first_valid) begin
            arvalid_m_inf <= 1'b1;
        end

    end

end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        arlen_m_inf <= 8'd0;
    end
    else if(n_state == IDLE) begin
        arlen_m_inf <= 8'd0;
    end
    else if(n_state == SE) begin
        arlen_m_inf <= se_len;

    end
    else if(n_state == PIC || n_state == HIST) begin
        arlen_m_inf <= pic_len;
    end

end


always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        rready_m_inf <= 1'b1;
    end
    else if(n_state == IDLE) begin

        rready_m_inf <= 1'b1;
    end

end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        araddr_m_inf <= se_baddr;
    end
    else if(n_state == IDLE) begin
        araddr_m_inf <= se_baddr;
    end
    else if(n_state == SE) begin
       
        // araddr_m_inf <= se_baddr;
        if(in_valid)begin
            araddr_m_inf <= (se_no << 4) + se_baddr;
        end
        
    end
    else if(n_state == HIST) begin
       
        // araddr_m_inf <= se_baddr;
        if(in_valid)begin
            araddr_m_inf <= (pic_no << 12) + pic_baddr;
        end
        
    end
    else if(n_state == PIC) begin
        //
        if(first_valid) begin//r_last?
            araddr_m_inf <= (t_pic_no << 12) + pic_baddr;
        end
        // else if(in_valid)begin
        //     araddr_m_inf <= (pic_no << 12) + pic_baddr;
        // end
    end

end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        awvalid_m_inf <= 1'b0;
    end
    else if(n_state == IDLE) begin
        awvalid_m_inf <= 1'b0;
    end
    else if(n_state == HIST) begin
        //
        if(awready_m_inf && awvalid_m_inf) begin//
            awvalid_m_inf <= 1'b0;
        end
        // else if(first_valid) begin
        //     arvalid_m_inf <= 1'b1;
        // end
        else if(in_valid) begin
            awvalid_m_inf <= 1'b1;
        end

    end
    else if(n_state == PIC) begin
        if(awready_m_inf && awvalid_m_inf) begin
            awvalid_m_inf <= 1'b0;
        end
        // else if(first_valid) begin
        //     awvalid_m_inf <= 1'b1;
        // end
        else if(first_valid) begin//
            awvalid_m_inf <= 1'b1;
        end
     

    end

end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        awaddr_m_inf <= pic_baddr;
    end
    else if(n_state == IDLE) begin
        awaddr_m_inf <= pic_baddr;
    end
    else if(n_state == HIST) begin
        if(in_valid) begin//r_last?
            awaddr_m_inf <= (pic_no << 12) + pic_baddr;
        end
    end
    else if(n_state == PIC) begin
        if(first_valid) begin//r_last?
            awaddr_m_inf <= (t_pic_no << 12) + pic_baddr;
        end
        // else if(in_valid)begin
        //     awaddr_m_inf <= (pic_no << 12) + pic_baddr;
        // end
        
    end

end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        wvalid_m_inf <= 1'b0;
    end
    else if(n_state == IDLE) begin

        wvalid_m_inf <= 1'b0;
    end
    else if(n_state == OUT) begin
        if(t_op == 2'd2) begin
            if(wlast_m_inf) begin
                wvalid_m_inf <= 1'b0;
            end
            else if(counter == 3'd3) begin
                wvalid_m_inf <= 1'b1;

            end
            
            // else if(!con_count)begin
            //     wvalid_m_inf <= 1'b1;
            // end
        end

        else begin
            if(wlast_m_inf) begin
                wvalid_m_inf <= 1'b0;
            end
            else begin
                wvalid_m_inf <= 1'b1;

            end

        end
        // if(wlast_m_inf) begin
        //     wvalid_m_inf <= 1'b0;
        // end
        // // else if(wvalid_m_inf && wready_m_inf && counter_2 == 8'd0) begin
        // //     wvalid_m_inf <= 1'b0;

        // // end
        // else if(counter == 3'd3) begin
        //     wvalid_m_inf <= 1'b1;

        // end
        // else begin
        //     wvalid_m_inf <= 1'b1;
        // end

        
        
        

    end
    

end
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        bready_m_inf <= 1'b0;
    end
    else if(n_state == IDLE) begin

        bready_m_inf <= 1'b0;
    end
    else if(n_state == OUT) begin
        bready_m_inf <= 1'b1;
        

    end
    

end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        wlast_m_inf <= 1'b0;
    end
    else if(n_state == IDLE) begin

        wlast_m_inf <= 1'b0;
    end
    else if(n_state == OUT) begin
        if(counter_2 == 8'd254 && wvalid_m_inf && wready_m_inf) begin
            wlast_m_inf <= 1'b1;
        end
        else begin
            wlast_m_inf <= 1'b0;
        end

    end
    

end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        awlen_m_inf <= 8'd0;
    end
    else if(n_state == IDLE) begin
        awlen_m_inf <= 8'd0;
    end
    else if(n_state == PIC || n_state == HIST) begin
        awlen_m_inf <= pic_len;
    end

end


always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        t_se_no <= 6'd0;

    end
    else if(n_state == IDLE) begin
        t_se_no <= 6'd0;  
    end
    else if(in_valid) begin
        t_se_no <= se_no;
    end
    // else if(n_state == SE) begin
    //     if(in_valid) begin
    //         t_se_no <= se_no;
    //     end

    // end
    // else if(n_state == PIC) begin
    //     se_a <= t_se_no;
    // end

end


always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        t_pic_no <= 4'd0;

    end
    else if(n_state == IDLE) begin
        t_pic_no <= 4'd0;  
    end
    else if(in_valid) begin
        t_pic_no <= pic_no;
    end
  

end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        t_op <= 2'd0;

    end
    else if(n_state == IDLE) begin
        t_op <= 2'd0;  
    end
    else if(in_valid) begin
        t_op <= op;
    end


end


always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        t_out_a <= 8'd0;
    end

    else if(n_state == IDLE) begin
        t_out_a <= 8'd0;  
    end
    else begin
        t_out_a <= out_a;
    end

end


always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        for(i = 0; i < 4; i = i + 1)begin
            for(j = 0; j < 4; j = j + 1)begin
                pic_arr[i][j] <= 128'd0;
            end
        end
    end
    else if(n_state == IDLE) begin
        for(i = 0; i < 4; i = i + 1)begin
            for(j = 0; j < 4; j = j + 1)begin
                pic_arr[i][j] <= 128'd0;
            end
        end  
    end
    else if(n_state == PIC) begin
        // if(rready_m_inf && rvalid_m_inf) begin//
        if(counter_2 <= 8'd3) begin
            pic_arr[counter_2][counter] <= rdata_m_inf;
        end
        else begin
            for(i = 0; i < 4; i = i + 1)begin
                for(j = 0; j < 3; j = j + 1)begin
                    pic_arr[i][j] <= pic_arr[i][j+1];
                end
            end
            pic_arr[0][3] <= pic_arr[1][0];
            pic_arr[1][3] <= pic_arr[2][0];
            pic_arr[2][3] <= pic_arr[3][0];
            pic_arr[3][3] <= rdata_m_inf;
        end
        // end
        

    end
   
end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        n_pic <= 1'b0;
    end
    else if(n_state == IDLE) begin
        n_pic <= 1'b0;
    end
    else if(n_state == SE) begin
        if(rready_m_inf && rvalid_m_inf) begin
            n_pic <= 1'b1;
        end
        

    end
   
end
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        se_arr <= 128'd0;
    end
    else if(n_state == IDLE) begin
        se_arr <= 128'd0;
    end
    else if(n_state == SE) begin
        if(rready_m_inf && rvalid_m_inf) begin
            se_arr <= rdata_m_inf;
        end
 

    end
   
end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        for(i = 0; i < 256; i = i + 1)begin
            cdf_c1[i] <= 13'd0;
        end
    end
    else if(n_state == IDLE) begin
        for(i = 0; i < 256; i = i + 1)begin
            cdf_c1[i] <= 13'd0;
        end 
    end
    else if(n_state == HIST) begin
        if(rready_m_inf && rvalid_m_inf) begin
            for(i = 0; i < 256; i = i + 1)begin
                cdf_c1[i] <= t_cdf_c1[i][15];
            end
        end
       
        

    end
   
end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        for(i = 0; i < 16; i = i + 1) begin
            n_temp_cdf[i] <= 21'd0;
        end
    end
    else if(n_state == IDLE) begin
        for(i = 0; i < 16; i = i + 1)begin
            n_temp_cdf[i] <= 21'd0;
        end 
    end
    else begin
   
        for(i = 0; i < 16; i = i + 1) begin
            n_temp_cdf[i] <= temp_cdf[i];
        end
        
       
        

    end
   
end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        for(i = 0; i < 16; i = i + 1) begin
            cdf_out[i] <= 8'd0;
        end
    end
    else if(n_state == IDLE) begin
        for(i = 0; i < 16; i = i + 1)begin
            cdf_out[i] <= 8'd0;
        end 
    end
    else if(counter_2 == 2'd0) begin
        if(wvalid_m_inf && wready_m_inf) begin
            for(i = 0; i < 16; i = i + 1) begin
                cdf_out[i] <= store2[i];
            end
        end
        else begin
            for(i = 0; i < 16; i = i + 1) begin
                cdf_out[i] <= store1[i];
            end

        end
    end

    else begin
        for(i = 0; i < 16; i = i + 1) begin
            cdf_out[i] <= n_temp_cdf2[i];
        end

    end
        
       
        

    
   
end
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        for(i = 0; i < 16; i = i + 1) begin
            store1[i] <= 8'd0;
            store2[i] <= 8'd0;
        end
    end
    else if(n_state == IDLE) begin
        for(i = 0; i < 16; i = i + 1)begin
            store1[i] <= 8'd0;
            store2[i] <= 8'd0;
        end 
    end
    else if(counter == 3'd2) begin
        for(i = 0; i < 16; i = i + 1)begin
            store1[i] <= n_temp_cdf2[i];
        end

    end
    else if(counter == 3'd3)begin
        for(i = 0; i < 16; i = i + 1)begin
            store2[i] <= n_temp_cdf2[i];
        end

    end
        
       
        

  
   
end
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        min_idx <= 8'd255;
    end
    else if(n_state == IDLE) begin
        min_idx <= 8'd255;
    end
    else if(n_state == HIST) begin
        if(rready_m_inf && rvalid_m_inf) begin
            if(min_erosion[0] < min_idx) begin
                min_idx <= min_erosion[0];
            end
        end
       
        

    end
   
end
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        his_min <= 13'd0;
    end
    else if(n_state == IDLE) begin
        his_min <= 13'd0;
    end
    else begin
        his_min <= 13'd4096 - cdf_min;
       
        

    end
   
end


always@(*) begin
case(c_state)
IDLE: begin
    out_oe = 1'b0;
    out_ce = 1'b0;
    out_we = 1'b1;
    out_a = 8'd0;
    if(in_valid) begin
        // if(first) begin
        if(op == 2'd2) begin
            n_state = HIST;
        end
        else begin
            n_state = SE;
        end
        // end
        // else begin
        //     n_state = PIC;
        // end
    end
    else begin
        n_state = IDLE;
    end
end
SE: begin
    out_oe = 1'b0;
    out_ce = 1'b0;
    out_we = 1'b1;
    out_a = 8'd0;
    if(n_pic) begin
        n_state = PIC;
    end
    else begin
        n_state = SE;
    end
end

PIC: begin
    out_oe = 1'b0;
    out_ce = 1'b0;
    if(counter_2 >= 8'd5) begin
        out_a = t_out_a + 8'd1;
        out_we = 1'b0;
    end
    else if(counter_2 >= 8'd4) begin
        if(counter >= 2'd1) begin
            out_a = t_out_a + 8'd1;
        end
        else begin
            out_a = 8'd0;
        end
        out_we = 1'b0;
    end
    
    else begin
        out_we = 1'b1;
        out_a = 8'd0;
    end
    // if(counter_2 >= 8'd5) begin
    //     out_a = t_out_a + 8'd1;
    // end
    // else begin
    //     out_a = 8'd0;
    // end
    if(counter_2 == 8'd0 && counter == 2'd0 && con_count) begin
        n_state = OUT;
    end
    else begin
        n_state = PIC;
    end

end

OUT: begin
    out_oe = 1'b0;
    out_ce = 1'b0;
    out_we = 1'b1;
    if(bvalid_m_inf && bresp_m_inf == 2'd0) begin
        n_state = IDLE;
    end
    else begin
        n_state = OUT;
    end
    if(t_op == 2'd2) begin
        if(counter < 3'd3) begin
            out_a = t_out_a + 8'd1;

        end
        else if((wvalid_m_inf && wready_m_inf)) begin
            out_a = t_out_a + 8'd1;
        end
        else begin
            out_a = t_out_a;
        end
    end
    else begin
        if((wvalid_m_inf && wready_m_inf)) begin
            out_a = t_out_a + 8'd1;
        end
        else begin
            out_a = 8'd0;
        end
    end

end
HIST: begin
    
    if(rready_m_inf && rvalid_m_inf) begin
        out_we = 1'b0;
        if(counter_2 >= 8'd1) begin
            out_a = t_out_a + 8'd1;
        end
        else begin
            out_a = 8'd0;
        end
        n_state = HIST;
    end
    else begin
        if(con_count) begin
            n_state = OUT;
        end
        else begin
            n_state = HIST;
        end
        out_we = 1'b1;
        out_a = 8'd0;
    end
    out_oe = 1'b0;
    out_ce = 1'b0;
    // if(counter_2 == 8'd0 && con_count) begin
    //     n_state = OUT;
    // end
    // else begin
    //     n_state = HIST;
    // end

end
default: begin
    out_oe = 1'b0;
    out_ce = 1'b0;
    out_we = 1'b1;
    out_a = 8'd0;
    n_state = IDLE;
end
endcase

end
endmodule