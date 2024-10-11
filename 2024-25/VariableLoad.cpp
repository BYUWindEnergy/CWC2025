/*
* Variable Load System (VariableLoad.cpp)
*
* This product was developed by Davi Cavinatto (Power Group) for use 
* in the main turbine control system. It contains 2 public functions
* that can be used to control and monitor the resistance values of
* the Variable Load System.
*
* Version 5.0 - November 2, 2023
* Recent changes
*   This version can select the best match among the 20 possible
*   resistance combinations.
*/

#include "Arduino.h"
#include "VariableLoad.h"

VariableLoad::VariableLoad(int SW0_PIN, int SW1_PIN, int SW2_PIN, int SW3_PIN, int SW4_PIN, int SW5_PIN, int SW6_PIN, int SW7_PIN, int SW_PARALLEL_PIN, int SW_SHORT_PIN, float R){
  
  pinMode(SW_PARALLEL_PIN, OUTPUT);
  pinMode(SW_SHORT_PIN, OUTPUT);
  pinMode(SW0_PIN, OUTPUT);
  pinMode(SW1_PIN, OUTPUT);
  pinMode(SW2_PIN, OUTPUT);
  pinMode(SW3_PIN, OUTPUT);
  pinMode(SW4_PIN, OUTPUT);
  pinMode(SW5_PIN, OUTPUT);
  pinMode(SW6_PIN, OUTPUT);
  pinMode(SW7_PIN, OUTPUT);

  _SW_PARALLEL_PIN = SW_PARALLEL_PIN;
  _SW_SHORT_PIN = SW_SHORT_PIN;
  _SW0_PIN = SW0_PIN;
  _SW1_PIN = SW1_PIN;
  _SW2_PIN = SW2_PIN;
  _SW3_PIN = SW3_PIN;
  _SW4_PIN = SW4_PIN;
  _SW5_PIN = SW5_PIN;
  _SW6_PIN = SW6_PIN;
  _SW7_PIN = SW7_PIN;
  _R = R;
  
  _resistance = R * 2.0;
  _resistance_table_pos = 0;
  _relaysOn = 0b001000100;
  
  _SW_PARALLEL_BITMASK = 0b100000000;
  _SW0_BITMASK = 0b010000000;
  _SW1_BITMASK = 0b001000000;
  _SW2_BITMASK = 0b000100000;
  _SW3_BITMASK = 0b000010000;
  _SW4_BITMASK = 0b000001000;
  _SW5_BITMASK = 0b000000100;
  _SW6_BITMASK = 0b000000010;
  _SW7_BITMASK = 0b000000001;
  
  setRelays(_relaysOn);
  digitalWrite(_SW_SHORT_PIN, LOW);

  // Create table of resistances vs relay bits
  createSingleResistanceTable(_resistances_table, _R);
}

void VariableLoad::shortLoad(int onOff){
  //1 is shorted, 0 is not shorted
	if (onOff==0){
		digitalWrite(_SW_SHORT_PIN, LOW);
	}
	else if (onOff == 1){
		digitalWrite(_SW_SHORT_PIN, HIGH);
	}
}

void VariableLoad::setBestResistanceMatch(float calculatedResistance){
	int indexOfBestMatch = 0;
	for (int i=0; i < 20; i++){
		if (_resistances_table[i].R >= calculatedResistance)
			indexOfBestMatch = i;
	}
	
	if (((_resistances_table[indexOfBestMatch].R - calculatedResistance) > (calculatedResistance  - _resistances_table[indexOfBestMatch+1].R)) && indexOfBestMatch < 19){
		indexOfBestMatch++;
	}
	
	_resistance_table_pos = indexOfBestMatch;
	_relaysOn = _resistances_table[_resistance_table_pos].relayBits;

	setRelays(_relaysOn);
}

void VariableLoad::createSingleResistanceTable(resTable *table, float R){
  table[0].R = R * 2; // 2.000
  table[0].relayBits = 0b001000100;
  table[1].R = R / 0.6; // 1.666
  table[1].relayBits = 0b000000000;
  table[2].R = R * 1.5; // 1.500
  table[2].relayBits = 0b000100010;
  table[3].R = R; // 1.000
  table[3].relayBits = 0b001010101;
  table[4].R = R / 1.2; // 0.833
  table[4].relayBits = 0b100000000;
  table[5].R = R * 0.75; // 0.750
  table[5].relayBits = 0b100100010;
  table[6].R = R / 1.5; // 0.666
  table[6].relayBits = 0b000010001;
  table[7].R = R / 1.6; // 0.625
  table[7].relayBits = 0b010001000;
  table[8].R = R * 0.6; // 0.600
  table[8].relayBits = 0b010101010;
  table[9].R = R / 2.0; // 0.500
  table[9].relayBits = 0b000110011;
  table[10].R = R / 2.5; // 0.400
  table[10].relayBits = 0b011001100;
  table[11].R = R * 0.375; // 0.375
  table[11].relayBits = 0b011101110;
  table[12].R = R / 3.0; // 0.333
  table[12].relayBits = 0b011011101;
  table[13].R = R / 3.2; // 0.3125
  table[13].relayBits = 0b110001000;
  table[14].R = R * 0.3; // 0.300
  table[14].relayBits = 0b110101010;
  table[15].R = R / 4.0; // 0.250
  table[15].relayBits = 0b011111111;
  table[16].R = R / 5.0; // 0.200
  table[16].relayBits = 0b111001100;
  table[17].R = R * 0.1875; // 0.1875
  table[17].relayBits = 0b111101110;
  table[18].R = R / 6.0; // 0.166
  table[18].relayBits = 0b111011101;
  table[19].R = R / 8.0; // 0.125
  table[19].relayBits = 0b111111111;
}

void VariableLoad::changeResistance(int direction){
  if ((direction == 0) && _resistance_table_pos < 19){
    _resistance_table_pos++;
  }
  else if ((direction == 1) && _resistance_table_pos > 0){
    _resistance_table_pos--;
  }
  
  _relaysOn = _resistances_table[_resistance_table_pos].relayBits;

  setRelays(_relaysOn);
}

void VariableLoad::updateResistance(){
  // All possible unique resistor values and combinations (20 in total)
  _resistance = _resistances_table[_resistance_table_pos].R;
}

void VariableLoad::setRelays(int newRelaysOn){

  // takes a newRelaysOn value and decides which switches to turn on/off
  if (newRelaysOn & _SW_PARALLEL_BITMASK)
    digitalWrite(_SW_PARALLEL_PIN, HIGH);
  else
    digitalWrite(_SW_PARALLEL_PIN, LOW);

  if (newRelaysOn & _SW0_BITMASK)
    digitalWrite(_SW0_PIN, HIGH);
  else
    digitalWrite(_SW0_PIN, LOW);

  if (newRelaysOn & _SW1_BITMASK)
    digitalWrite(_SW1_PIN, HIGH);
  else
    digitalWrite(_SW1_PIN, LOW);

  if (newRelaysOn & _SW2_BITMASK)
    digitalWrite(_SW2_PIN, HIGH);
  else
    digitalWrite(_SW2_PIN, LOW);

  if (newRelaysOn & _SW3_BITMASK)
    digitalWrite(_SW3_PIN, HIGH);
  else
    digitalWrite(_SW3_PIN, LOW);

  if (newRelaysOn & _SW4_BITMASK)
    digitalWrite(_SW4_PIN, HIGH);
  else
    digitalWrite(_SW4_PIN, LOW);

  if (newRelaysOn & _SW5_BITMASK)
    digitalWrite(_SW5_PIN, HIGH);
  else
    digitalWrite(_SW5_PIN, LOW);

  if (newRelaysOn & _SW6_BITMASK)
    digitalWrite(_SW6_PIN, HIGH);
  else
    digitalWrite(_SW6_PIN, LOW);

  if (newRelaysOn & _SW7_BITMASK)
    digitalWrite(_SW7_PIN, HIGH);
  else
    digitalWrite(_SW7_PIN, LOW);
  // updates relaysOn with newRelaysOn value
  updateResistance();
}

float VariableLoad::getResistance(){
  return _resistance; 
}

