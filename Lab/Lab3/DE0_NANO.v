//=======================================================
// ECE3400 Fall 2017
// Lab 3: Template top-level module
//
// Top-level skeleton from Terasic
// Modified by Claire Chen for ECE3400 Fall 2017
//=======================================================

`define ONE_SEC 25000000

module DE0_NANO(

	//////////// CLOCK //////////
	CLOCK_50,

	//////////// LED //////////
	LED,

	//////////// KEY //////////
	KEY,

	//////////// SW //////////
	SW,

	//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
	GPIO_0_D,
	GPIO_0_IN,

	//////////// GPIO_0, GPIO_1 connect to GPIO Default //////////
	GPIO_1_D,
	GPIO_1_IN,
);

	 //=======================================================
	 //  PARAMETER declarations
	 //=======================================================

	 localparam ONE_SEC = 25000000; // one second in 25MHz clock cycles
	 
	 //=======================================================
	 //  PORT declarations
	 //=======================================================

	 //////////// CLOCK //////////
	 input 		          		CLOCK_50;

	 //////////// LED //////////
	 output		     [7:0]		LED;

	 /////////// KEY //////////
	 input 		     [1:0]		KEY;

	 //////////// SW //////////
	 input 		     [3:0]		SW;

	 //////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
	 inout 		    [33:0]		GPIO_0_D;
	 input 		     [1:0]		GPIO_0_IN;

	 //////////// GPIO_0, GPIO_1 connect to GPIO Default //////////
	 inout 		    [33:0]		GPIO_1_D;
	 input 		     [1:0]		GPIO_1_IN;

    //=======================================================
    //  REG/WIRE declarations
    //=======================================================
    reg         CLOCK_25;
    wire        reset; // active high reset signal 

    wire [9:0]  PIXEL_COORD_X; // current x-coord from VGA driver
    wire [9:0]  PIXEL_COORD_Y; // current y-coord from VGA driver
	 
	 
    reg [7:0]  PIXEL_COLOR;   // input 8-bit pixel color for current coords
	 
	 reg [24:0] led_counter; // timer to keep track of when to toggle LED
	 reg 			led_state;   // 1 is on, 0 is off
	 reg 			movement_state;   // keeps track of whether the data is new
	 wire  	   MovementX;  // True if Robot Moved X
	 wire		   MovementY;  // True if Robot Moved Y
	 reg		   PositionX; // True if +X, False is -X
	 reg	      PositionY; // True if +Y, False is -Y
	 reg [1:0]			PositionXandY;
	 
	 // The Maze Array. We always start in the middle of array.
	 // This array is big enough for worst case of robot starting in a corner.
	 reg [1:0] Array [1:0] [1:0];
	 reg x;
	 reg y;
	 reg [1:0] i;
	 reg [1:0] j;
	 
	 initial begin
	 
			for (i=1'd0; i<1'd1; i = i + 1'd1) begin
				for (j=1'd0; j<1'd1; j = j + 1'd1) begin
					Array[i][j] = 1'b0; // Initialize everything to 0
				end
			end
			//Array[1'b0][1'b0] = 1'b0; // This is the starting point
			
			x = 1'd0;
			y = 1'd0;
			movement_state = 1'b1;
			
	 end
	 
	 
	 
    // Module outputs coordinates of next pixel to be written onto screen
    VGA_DRIVER driver(
		  .RESET(reset),
        .CLOCK(CLOCK_25),
        .PIXEL_COLOR_IN(PIXEL_COLOR),
        .PIXEL_X(PIXEL_COORD_X),
        .PIXEL_Y(PIXEL_COORD_Y),
        .PIXEL_COLOR_OUT({GPIO_0_D[9],GPIO_0_D[11],GPIO_0_D[13],GPIO_0_D[15],GPIO_0_D[17],GPIO_0_D[19],GPIO_0_D[21],GPIO_0_D[23]}),
		  .H_SYNC_NEG(GPIO_0_D[7]),
        .V_SYNC_NEG(GPIO_0_D[5])
    );
	 
	 assign reset = ~KEY[0]; // reset when KEY0 is pressed
	 
	 //assign MovementX = GPIO_0_D[30];  //True if Robot Moved X
	 //assign MovementY = GPIO_0_D[32];  //True if Robot Moved Y
	 
	 
always @(posedge CLOCK_25) begin
/*
if (MovementX == 1'b1 && movement_state == 1'b1) begin
	
		movement_state <= 1'b0; // Perform array update once
		
		// Update Array with new position
		//if (!PositionX) begin // Moved -X
		//x <= x - 1'd1;
		//Array[x][y] <= 1'b1;
		//end
		if (PositionX == 1'b1) begin // Moved +X
		x <= x + 1'd1;
		Array[x][y] <= 1'b1;
		end
end */
 PositionX = GPIO_0_D[31]; // True if +X, False is -X
 PositionY = GPIO_0_D[33]; // True if +Y, False is -Y
 PositionXandY = {PositionX, PositionY};
	case(PositionXandY)
		2'b00: begin
		Array[x][y] = 1'b0;
		x = x*1'b0;
		y = y*1'b0;
		Array[x][y] = 1'b1; end
		2'b01: begin
		Array[x][y] = 1'b0;
		x = x*1'b0;
		x = x+1'b1;
		y=y*1'b0;
		Array[x][y] = 1'b1; end
		2'b10: begin
		Array[x][y] = 1'b0;
		x = x*1'b0;
		y = y*1'b0;
		y = y + 1'b1;
		Array[x][y] = 1'b1; end
		2'b11: begin
		Array[x][y] = 1'b0;
		x = x*1'b0;
		x = x + 1'b1;
		y = y*1'b0;
		y = y + 1'b1;
		Array[x][y] = 1'b1; end
		default: Array[x][y] = 1'b1;
	endcase

	/*
if (movement_state == 1'b1) begin

		movement_state <= 1'b0; // Perform array update once
		
		if (!PositionY) begin // Moved -Y
		y <= y - 1'd1;
		Array[x][y] <= 1'b1;
		end
		if (PositionY) begin // Moved +Y
		y <= y + 1'd1;
		Array[x][y] <= 1'b1;
		end
end*/
	
	// a way to traverse this array and make boxes appear for elements which are true

	// (0,0)
	if (PIXEL_COORD_X > (10'd60-10'd30) && PIXEL_COORD_X < (10'd60+10'd30) && PIXEL_COORD_Y > (10'd60-10'd30) && PIXEL_COORD_Y < (10'd60+10'd30)) begin
		if(Array[1'd0][1'd0])begin
		PIXEL_COLOR = 8'b000_001_11; end
		else begin
		PIXEL_COLOR = 8'b101_010_11; end
		// 
	end
	// (0,1)
	else if (PIXEL_COORD_X > (10'd60-10'd30) && PIXEL_COORD_X < (10'd60+10'd30) && PIXEL_COORD_Y > (10'd60*2-10'd30) && PIXEL_COORD_Y < (10'd60*2+10'd30)) begin
		if(Array[1'd0][1'd1])begin
		PIXEL_COLOR = 8'b000_001_11; end
		else begin
		PIXEL_COLOR = 8'b101_010_11; end
	end
	// (1,0)
	else if (PIXEL_COORD_X > (10'd60*2-10'd30) && PIXEL_COORD_X < (10'd60*2+10'd30) && PIXEL_COORD_Y > (10'd60-10'd30) && PIXEL_COORD_Y < (10'd60+10'd30)) begin
		if(Array[1'd1][1'd0])begin
		PIXEL_COLOR = 8'b000_001_11; end
		else begin
		PIXEL_COLOR = 8'b101_010_11; end
	end
	// (1,1)
	else if (PIXEL_COORD_X > (10'd60*2-10'd30) && PIXEL_COORD_X < (10'd60*2+10'd30) && PIXEL_COORD_Y > (10'd60*2-10'd30) && PIXEL_COORD_Y < (10'd60*2+10'd30)) begin
		if(Array[1'd1][1'd1])begin
		PIXEL_COLOR = 8'b000_001_11; end
		else begin
		PIXEL_COLOR = 8'b101_010_11; end 
	end
	
	else begin
		PIXEL_COLOR = 8'b101_010_11;
	end
	
end


	 
	 assign LED[0] = led_state;
	 
    //=======================================================
    //  Structural coding
    //=======================================================
 
	 // Generate 25MHz clock for VGA, FPGA has 50 MHz clock
    always @ (posedge CLOCK_50) begin
        CLOCK_25 <= ~CLOCK_25; 
    end // always @ (posedge CLOCK_50)
	
	 // Simple state machine to toggle LED0 every one second
	 always @ (posedge CLOCK_25) begin
		  if (reset) begin
				led_state   <= 1'b0;
				led_counter <= 25'b0;
		  end
		  
		  if (led_counter == ONE_SEC) begin
				led_state   <= ~led_state;
				led_counter <= 25'b0;
		  end
		  else begin	
				led_state   <= led_state;
				led_counter <= led_counter + 25'b1;
		  end // always @ (posedge CLOCK_25)
	 end
	 

endmodule
