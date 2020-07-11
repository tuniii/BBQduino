/*      Last edit: Jan 11, 2014
 *
 * BBQduino Maverick 732 BBQ Wireless Thermometer Sniffer v0.1x
 *     Also verified to work properly with Ivation Model #IVAWLTHERM BBQ Thermometer
 *
 *    (c) 2014 B. Tod Cox, John Cox
 *    (c) 2020 Martin Koerner
 *
 * BBQduino is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * BBQduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA
 *
 * Receives temperature data from a Maverick 732 (and clones) Wireless BBQ Thermometer
 * and outputs to an Adafruit HT1632 LED matrix as well as providing temps via a web server.
 *
 * Manchester decoding and interrupt handler based in part on code and ideas from  
 *         http://www.practicalarduino.com/projects/weather-station-receiver
 *         http://kayno.net/2010/01/15/arduino-weather-station-receiver-shield/
 *         https://forums.adafruit.com/viewtopic.php?f=8&t=25414&start=0
 *         http://wiki.openpicus.com/index.php?title=Wifi_bbq
 *     Web server & HTML/Canvas gauges sources:
 *         http://startingelectronics.com/tutorials/arduino/ethernet-shield-web-server-tutorial/  
 *         https://github.com/Mikhus/canv-gauge
 *
 *      Licenses of above works are include by reference.
 */

void bbq433Init();
boolean bbq433CheckData(uint16_t delta);
int bbq433GetTemperature(uint8_t index);
