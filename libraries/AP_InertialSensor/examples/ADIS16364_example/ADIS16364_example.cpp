////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Analog Devices, Inc.
//  June 2012
//  By: Adam Gleason, and Brian Holford
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16364_example.ino
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This file is part of Interfacing ADIS16364 with Arduino example.
//
//  Interfacing ADIS16364 with Arduino example is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  Interfacing ADIS16364 with Arduino example is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser Public License for more details.
//
//  You should have received a copy of the GNU Lesser Public License
//  along with Interfacing ADIS16364 with Arduino example.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  NOTES:
//  This software package is an example of how to interface an ADIS16364 with Arduino Uno, or Mega 2560.
////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "ADIS16364.h"
#include <unistd.h>

// Instantiate ADIS16364 class as iSensor with CS pin 53
// ****Use this for Mega 2560 setup****
//ADIS16364 iSensor(53);

// Instantiate ADIS16364 class as iSensor with CS pin 10
ADIS16364 iSensor(10);

int main(void)
{

  // Print out debug information
  // ****NOTE****
  // this will mess up the plotting program if you do this at the same time
  // so only use it if you want to verify setup before plotting 
  
  while(1)
  {
     // Perform burst read on iSensor
     iSensor.debug();
     sleep(2);
    
  }
  return 0;
  
}
