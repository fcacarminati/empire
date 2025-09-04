#define BUZZER_PIN 9 
float pitch[12] = {16.35, 17.32, 18.35, 19.45, 20.60, 21.83, 23.12, 24.50, 25.96, 27.50, 29.14, 30.87};
char* note[12] = {"C", "C#/Db", "D", "D#/Eb", "E", "F" "F#/Gb", "G", "G#/Ab", "A", "A#/Bb", "B"};
//                 0.     1.     2.     3.     4.   5.    6.     7.     8.     9.     10.    11. 
char* noteIt[12] = {"Do", "Do#/Reb", "Re", "Re#/Mib", "Mi", "Fa" "Fa#/Solb", "Sol", "Sol#/Lab", "La", "La#/Sib", "Si"};

static const int mainOctave = 4;

int melody[] =    { 0, 2, 4, 0, 0, 2, 4, 0, 4, 5, 7, 4, 5, 7, 7, 9, 7, 5, 4, 0, 7, 9, 7, 5, 4, 0, 0, 7, 0, 0, 7, 0
	 // Notes goes here 
}; 
int durations[] = { 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 2, 4, 4, 2, 8, 8, 8, 8, 4, 4, 8, 8, 8, 8, 4, 4, 4, 4, 2, 4, 4, 2
	 // Notes duration goes here 
}; 

int octaves [] =  { 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 7, 7, 6
  // Octave goes here
};

void setup() 
{ 
	 pinMode(BUZZER_PIN, OUTPUT); 
} 
void loop() 
{ 
	delay(2000);
	 int size = sizeof(durations) / sizeof(int); 
	 for (int note = 0; note < size; note++) { 
	   //to calculate the note duration, take one second divided by the note type. 
	   //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc. 
	   int duration = 2000 / durations[note]; 
	   tone(BUZZER_PIN, int(pitch[melody[note]]*octaves[note]+0.5));//, duration); 
	   //to distinguish the notes, set a minimum time between them. 
	   //the note's duration + 30% seems to work well: 
	   int pauseBetweenNotes = duration * 1.2; 
	   delay(pauseBetweenNotes); 
	   //stop the tone playing: 
	   noTone(BUZZER_PIN); 
	 } 
} 
