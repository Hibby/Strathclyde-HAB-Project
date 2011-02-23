This is the Arduino New Soft Serial Fork!

To run, just download the file inside gps_ex, and make sure it is placed within The Arduino sketchbook folder, in a folder named gps_ex_nss also.
Ensure that NewSoftSerial from http://arduiniana.org/libraries/newsoftserial/ is in your libraries also.

Then restart arduino, and it will appear in File:Sketchbook as gps_ex_nss!

It should start, output some config messages and begin to search for GPS satellites.

Given 30-60 seconds, a lock should be found and useful data will appear.

The GPS TX is connected to pin 10 on the board and the GPS RX is connected to pin 9 on the board, however this is not a requirement, just change

NewSoftSerial nss(10,9); 