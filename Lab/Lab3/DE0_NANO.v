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
	 wire [1:0]  Movement;  // True if Robot Moved
	 wire [1:0]  PositionX; // True if +X, False is -X
	 wire [1:0]  PositionY; // True if +Y, False is -Y
	 
    reg [7:0]  PIXEL_COLOR;   // input 8-bit pixel color for current coords
	 
	 reg [24:0] led_counter; // timer to keep track of when to toggle LED
	 reg 			led_state;   // 1 is on, 0 is off
	 reg 			movement_state = 1'd0;   // keeps track of whether the data is new
	 
	 // The Maze Array. We always start in the middle of array.
	 // This array is big enough for worst case of robot starting in a corner.
	 reg [48:0] Array [6:0] [6:0];
	 reg [6:0] x;
	 reg [6:0] y;
	 reg [6:0] i;
	 reg [6:0] j;
	 
	 initial begin
	 
			for (i=1; i<7; i = i + 1) begin
				for (j=1; j<7; j = j + 1) begin
					Array[i][j] = 1'd0; // Initialize everything to 0
				end
			end
			Array[4][4] = 1'd1; // This is the starting point
			
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
	 
	 assign Movement = GPIO_0_D[29];  //True if Robot Moved
	 assign PositionX = GPIO_0_D[31]; // True if +X, False is -X
	 assign PositionY = GPIO_0_D[33]; // True if +Y, False is -Y
	 
	 
always @(*) begin

if (Movement != 1'b0) begin

	if (movement_state == 1'd1) begin
	
		movement_state = 1'd0; // Perform array update once
		
		// Update Array with new position
		if (!PositionX) begin // Moved -X
		x = x - 1'd1;
		Array[x][y] = 1'd1;
		end
		if (PositionX) begin // Moved +X
		x = x + 1'd1;
		Array[x][y] = 1'd1;
		end
		if (!PositionY) begin // Moved -Y
		y = y - 1'd1;
		Array[x][y] = 1'd1;
		end
		if (PositionY) begin // Moved +Y
		y = y + 1'd1;
		Array[x][y] = 1'd1;
		end
		else begin
		// Should never be here
		end
		
		
	end
	
	// a way to traverse this array and make boxes appear for elements which are true
	
	
	 if (PIXEL_COORD_X > 10'd64 && PIXEL_COORD_X < 10'd120 && PIXEL_COORD_Y > 10'd64 && PIXEL_COORD_Y < 10'd120) begin
			PIXEL_COLOR = 8'b111_111_11; // 
	 end
	 
	 else begin
			PIXEL_COLOR = 8'b111_000_11; // Green
	 end
end

		else begin
		PIXEL_COLOR = 8'b111_000_11; // Green
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
