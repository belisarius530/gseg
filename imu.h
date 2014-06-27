//======================================================================================
/** \file imu.h
 *    This file contains an IMU driver.
 *
 *  License:
 *    This file is copyright 2014 by E Bacon, M Berry, J Fish, S Oh and released under the Lesser GNU 
 *    Public License, version 2. It intended for educational use only, but its use
 *    is not limited thereto. */
/*    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 *    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUEN-
 *    TIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
 *    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 *    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 *    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
//======================================================================================


#ifndef _IMU_H_
#define _IMU_H_

#include <stdlib.h>
#include <avr/io.h>  
class imu
{
	
protected:
	/// use this pointer to the serial port for debug info
	emstream* ptr_to_serial;
	int16_t x_accl;
	int16_t y_accl;
	int16_t z_accl;

	int16_t x_gyro;
	int16_t y_gyro;
	int16_t z_gyro;

	int16_t x_mag;
	int16_t y_mag;
	int16_t z_mag;
        
        float gyro_roll_angle;
        float gyro_pitch_angle;
        
        float roll_angle;
        float pitch_angle;
        float yaw_angle;

public:
	// The constructor sets up the IMU driver for use. The "= NULL" part is a
	// default parameter, meaning that if that parameter isn't given on the line
	// where this constructor is called, the compiler will just fill in "NULL".
	// In this case that has the effect of turning off diagnostic printouts
	imu(void);
	
	void get_position(void);
	int16_t x_gyro_val(void);
	int16_t x_accl_val(void);
	int16_t x_mag_val(void);
	
	int16_t z_gyro_val(void);
	int16_t z_accl_val(void);
	int16_t z_mag_val(void);
	
	int16_t y_gyro_val(void);
	int16_t y_accl_val(void);
	int16_t y_mag_val(void);
	
	
	void gyro_init(void);
	void gyro_position(void);
	void accl_init(void);
	void accl_position(void);
	void mag_init(void);
	void mag_position(void);
	
    void compute_attitude(float);
    float pitch(void);
    float roll(void);
	float arctan(float);
};

/* EXAMPLE CODE :
 * imu1 = imu(ptr_to_serial);
 *
 * ---- In the loop ---
 * imu1.get_position();
 * DBG(ptr_to_serial, "The x accelerometer value is: " << imu.x_accl_val());
 * DBG(ptr_to_serial, "The x gyro value is: " << imu.x_gyro_val());
 *
 */
#endif