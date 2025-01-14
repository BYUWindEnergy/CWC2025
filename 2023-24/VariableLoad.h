/*
* Variable Load System (variableLoad.h)
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

#ifndef VariableLoad_h
#define VariableLoad_h

#include "Arduino.h"

class VariableLoad {
	public:
    // Selects arduino pins that match the variable load circuit and saves the value of the base resistor
		VariableLoad(int SW0_PIN, int SW1_PIN, int SW2_PIN, int SW3_PIN, int SW4_PIN, int SW5_PIN, int SW6_PIN, int SW7_PIN, int SW_PARALLEL_PIN, int SW_SHORT_PIN, float R);
		// If direction=0, lowers resistance by one step
    // If direction=1, raises resistance by one step
    void changeResistance(int direction);
	// Short load if 1, normal operation if 0
	void shortLoad(int onOff);
    // Lets the user know which resistance is currently selected
    float getResistance();
	void setBestResistanceMatch(float calculatedResistance);
	private:
    typedef struct {
                float R;
                int relayBits;
               } resTable;

		void setRelays(int newRelaysOn);
		void updateResistance();
    void createSingleResistanceTable(resTable *table, float R);
		int _SW_PARALLEL_PIN;
		int _SW_SHORT_PIN;
		int _SW0_PIN;
		int _SW1_PIN;
		int _SW2_PIN;
		int _SW3_PIN;
		int _SW4_PIN;
		int _SW5_PIN;
		int _SW6_PIN;
		int _SW7_PIN;
		float _R;
		float _resistance;
    int _resistance_table_pos;
		int _relaysOn;
		int _SW_PARALLEL_BITMASK;
		int _SW0_BITMASK;
		int _SW1_BITMASK;
		int _SW2_BITMASK;
		int _SW3_BITMASK;
		int _SW4_BITMASK;
		int _SW5_BITMASK;
		int _SW6_BITMASK;
		int _SW7_BITMASK;
    resTable _resistances_table[20];
};


#endif