#include <LiquidCrystal.h>
#include <TimerOne.h>
#include <EEPROM.h>

#define rs  A0      // А0-А5 - lcd
#define en  A1 
#define d4  A2 
#define d5  A3 
#define d6  A4 
#define d7  A5
#define INC 2       // encoder increment DT
#define DECR 4      // encoder decrement CLK
#define BUT 3       // encoder button  SW
#define STEP 12     // impulse to step driver
#define DIR 11      // step driver direction
#define termodat 10  // termodat - https://www.termodat.ru/
#define enable 9  // enable signal to driver


//////////////////////////////////////// Variables used //////////////////////////////////////////////////
unsigned long start_time, t, lcd_initialize_counter;

byte eeprom_data[] = {10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int calibration_coeff, current_calibration;
double value_for_print; 

byte i = 1, mode = 0, j_step = 1, point = 10, cur_item = 0, number_of_steps = 0;

int timer_handler_counter = 0, incremental_var = 0, current_var = 10, interrupt_pause; 

int pulling_speeds[10], pulling_lengths[10];

boolean is_button_pressed = false, is_menu_active = false, timer_interrupt_permission = false, 
length_control  = false, is_speed_set = true, is_multi_mode_settings = false,
is_need_to_set_number_of_steps = true;

char* menu_points[] = {"Main screen", "Select mode", "Multi details", "Default params"};   // points of menu
char* mode_points[] = {"Mono", "Multi", "Calibration", "Current params", "Main screen"};   // points of mode

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);  


void setup() {
   
   // Initial modes and level on PINs
   pinMode(DIR, OUTPUT);
   pinMode(STEP, OUTPUT);
   pinMode(enable, OUTPUT);
   pinMode(INC, INPUT);
   pinMode(DECR, INPUT);
   pinMode(DIR, OUTPUT);
   pinMode(termodat, INPUT);     
   digitalWrite(termodat, HIGH);
   dirDOWN();
   //

   if (EEPROM[0] == 255) {
    EEPROM.put(0, eeprom_data);
    }
   EEPROM.get(0, eeprom_data);
   EEPROM.get(22, calibration_coeff);
   current_calibration = calibration_coeff;
   current_var = eeprom_data[0];
   incremental_var = current_var;
   const int eeprom_data_length = sizeof(eeprom_data) / sizeof(eeprom_data[0]);
   
   Timer1.initialize(1000);            // initialize timer 1
   Timer1.attachInterrupt(timer_interrupt_handler);        // set handler for timer interrupt - timer_interrupt_handler 
   
   // Initialize external interrupts, handlers: encoder and button
   attachInterrupt(0, encoder, FALLING);
   attachInterrupt(1, button, FALLING);    

   lcd.begin(16, 2);
}

void loop() {

  if(is_pulling_allowed()) { // when pulling is proccessing 
    enabled(); // step driver moving enabled
      
    if(number_of_steps == 0) { // if is mode "Mono" - start_of_block
      timer_interrupt_permission = true; // permission to execute the timer interrupt handler 
      speed_to_impulses(current_var);
      
      while(true) {  // while the mono pulling - start of block
        //dirDOWN();
        
        lcd.clear();
        lcd.print("V=");
        lcd.print(to_double(current_var));
        lcd.print("mm/h"); 
        lcd.setCursor(1,1);
        lcd.print("Pulling");
        
        if(digitalRead(termodat) != LOW) {
          break;
          }
          
        delay(100);
        
        } // while the mono pulling - end of block
      
      } // if is mode "Mono" - end_of_block
      
    else { // if is mode "Multi" - start_of_block
      j_step = 0;
      timer_interrupt_permission = true;   // permission to execute the handler body 
      incremental_var = 0;
      
      speed_to_impulses(pulling_speeds[j_step]);
      
      start_time = millis();          // time when current step is beginning
      
      while(true) {  // while the multi pulling - start of block
        dirDOWN();
        
        t = (long)(millis() - start_time)/1000; // time elapsed from the beginning of the current step 
        
        if (t * pulling_speeds[j_step]/3600 > 10 * pulling_lengths[j_step]) { // if current pulling length is more than pulling_lenghts[j_step]
          j_step++; // next step
          incremental_var = j_step;       
          speed_to_impulses(pulling_speeds[j_step]); // change current speed of pulling
          start_time = millis();
        }
        
        if(j_step >= number_of_steps) { 
          length_control  = true;
          break;
          }
          
        circular_listing(0, number_of_steps); // limit value
        
        lcd.clear();

        // print current number of step, quantity of steps, speed and length of current state, you can look speeds and lengths of all steps by changing position of encoder
        lcd.print("PulStep ");
        lcd.print(j_step + 1);
        lcd.print("/");
        lcd.print(number_of_steps);
        lcd.setCursor(0,1);
        lcd.print(incremental_var);
        lcd.print(":v=");
        value_for_print = to_double(pulling_speeds[incremental_var]);
        lcd.print(value_for_print); 
        lcd.print(" d=");
        value_for_print = to_double(pulling_lengths[incremental_var]);
        lcd.print(value_for_print);

        if(digitalRead(termodat) != LOW) {              // if there is signal from termodat
          break;
          }
        
        delay(300);
         
      } // while the multi pulling - end of block
      
    } // if is mode "Multi" - end_of_block
    
  }
   else {         // when there is no pulling
/******************************************** MENU *****************************************************/   
   timer_interrupt_permission = false; // permission to execute the interrupt handler body 
   disabled(); // step driver moving disabled
   
   digitalWrite(termodat, HIGH);


   if(is_menu_active) {  //open menu list - start of block

    circular_listing(0, 3); // circular listing of the menu list
    
    delay(100);
    lcd.clear();
    lcd.print(menu_points[incremental_var]); // points_of_menu
    
    if(is_button_pressed){ // if button pressed, choose current item of menu list
      is_button_pressed = false; 
      point = incremental_var;              
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(point == 1) { // choosing pulling mode - start of block
      incremental_var = 0;
      point = 10;
      is_menu_active = false;

      while(true) {  // circular while choosing pulling mode - start of block
        
        circular_listing(0, 1); // circular listing of mode list
        lcd.clear();
        lcd.print(mode_points[incremental_var]); // modes list
        delay(100);

        if(is_button_pressed){ // if button pressed, choose current mode
         is_button_pressed = false; 
         mode = incremental_var;
         break;
        }
        
      } // circular while choosing pulling mode - end of block
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      
      if(mode == 1) { // multi mode - start of block
        incremental_var = 1;
        cur_item = 0;
        mode = 0;
        
        is_speed_set = true;
        is_multi_mode_settings = true;
        is_need_to_set_number_of_steps = true;
        
        while(is_need_to_set_number_of_steps) { // set the number of steps - start of block
          delay(100);
          lcd.clear();
          if (incremental_var == -1) {
            lcd.print("Set def params");
            }
           else {
            lcd.print("NSteps: ");
            lcd.print(incremental_var);
           }
          limit_listing(-1, 10); // limit value

          if(is_button_pressed) {
            is_need_to_set_number_of_steps = false;
            is_button_pressed = false;
            if (incremental_var == -1) {
              set_default_multi_parameters();
              cur_item = number_of_steps;
            }
            else {
              number_of_steps = incremental_var;
              cur_item = 0;
            }
            incremental_var = 1;
          }
          
        } // set the number of steps - end of block
       
        while(cur_item < number_of_steps) {  // set speed and length for all steps - start of block
          if(is_speed_set) { // set pulling speed of current step - start of block
            
            limit_listing(0, 200); // limit value
            lcd.clear();
            value_for_print = to_double(incremental_var);
            print_array_element("v", cur_item + 1, value_for_print); 
            delay(100);
            
            if(is_button_pressed){ // set pulling speed for i's step
              pulling_speeds[cur_item] = incremental_var;
              incremental_var = 1;
              is_button_pressed = false;
              is_speed_set = false;
            }
            
          } // set pulling speed of current step - end of block
          
          else { // set pulling length of current step - start of block
            
            limit_listing(0, 200); // limit value
            delay(100);
            lcd.clear();
            value_for_print = to_double(incremental_var);
            print_array_element("d", cur_item + 1, value_for_print); 
            
            
            if(is_button_pressed){
              pulling_lengths[cur_item] = incremental_var; // set pulling length for i's step
              incremental_var = 1;
              is_button_pressed = false;
              is_speed_set = true;
              cur_item++;
            }
          } // set pulling length of current step - end of block
          
          
        } // set speed and length for all steps - end of block
       
      } // multi mode - end of block
      
    } // choosing pulling mode - end of block
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(point == 2) {  // menu point "Show Multi mode details" - start of block
      incremental_var = 1;
      point = 10;
      while(true){ // preset parameters of multi mode - start of block
        delay(100);
        
        lcd.clear();
        value_for_print = to_double(pulling_speeds[incremental_var]);
        print_array_element("v", incremental_var, value_for_print);
        
        lcd.setCursor(0,1);
        value_for_print = to_double(pulling_lengths[incremental_var]);
        print_array_element("d", incremental_var, value_for_print);

        circular_listing(0, number_of_steps); // limit value
       
        if(is_button_pressed){
          is_button_pressed = false;
          break;
          }
          
        } // preset parameters of multi mode - end of block
      
      }  // menu point "Show Multi mode details" - end of block
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      if (point == 3) { // default parameters - start of block
        incremental_var = 0;
        point = 10;
        is_menu_active = false;

        while(true) {  // circular while choosing default mode - start of block
          
          circular_listing(0, 4); // circular listing of mode list 
          delay(100);
          lcd.clear();
          lcd.print(mode_points[incremental_var]); // modes list
  
          if(is_button_pressed){ // if button pressed, choose current mode
           is_button_pressed = false; 
           mode = incremental_var;
           break;
          }
        }

        
        if (mode == 0) { // set default mono mode speed
          mode = 10;
          incremental_var = eeprom_data[0];
          while(true) {
            delay(100);
            lcd.clear(); 
            lcd.print("v = ");
            lcd.print(to_double(incremental_var));
            lcd.print(" mm/h ");
            
            limit_listing(0, 200); // limit value
            
            if(is_button_pressed) { // if button pressed set default mono mode speed
              eeprom_data[0] = incremental_var;
              current_var = incremental_var;
              is_button_pressed = false;
              EEPROM.put(0, eeprom_data);
              break;
            }
          }  
        } // set default mono mode speed

         
        if(mode == 1) { // set multi default parameters - start of block
          incremental_var = eeprom_data[1];
          cur_item = 0;
          mode = 10;
          
          is_speed_set = true;
          is_multi_mode_settings = true;
          
          while(is_need_to_set_number_of_steps) { // set the number of steps - start of block
            delay(100);
            lcd.clear();
            lcd.print("defNSteps: ");
            lcd.print(incremental_var);
            
            limit_listing(0, 10); // limit value
  
            if(is_button_pressed) {
              eeprom_data[1] = incremental_var;
              incremental_var = 10;
              is_need_to_set_number_of_steps = false;
              is_button_pressed = false;
            }
            
          } // set the number of steps for default params - end of block
          
          cur_item = 0;
          while(cur_item < eeprom_data[1]) {  // set speed and length for all steps - start of block
            if(is_speed_set) { // set default pulling speed of current step - start of block
              
              limit_listing(0, 200); // limit value
              
              lcd.clear();
              value_for_print = to_double(incremental_var);
              print_array_element("v", cur_item + 1, value_for_print); 
              delay(100);
              
              if(is_button_pressed){ // set pulling speed for i's step
                eeprom_data[2*cur_item+2] = incremental_var;
                incremental_var = 1;
                is_button_pressed = false;
                is_speed_set = false;
              }
              
            } // set default pulling speed of current step - end of block if
            
            else { // set default pulling length of current step - start of block
              
              limit_listing(0, 200); // limit value
              
              lcd.clear();
              value_for_print = to_double(incremental_var);
              print_array_element("d", cur_item + 1, value_for_print); 
              delay(100);

              
              if(is_button_pressed){
                eeprom_data[2*cur_item+3] = incremental_var; // set pulling length for i's step
                incremental_var = 1;
                is_button_pressed = false;
                is_speed_set = true;
                cur_item++;
              }
            } // set default pulling length of current step - end of block else
            
            
          } // set speed and length for all steps - end of block while

          EEPROM.put(0, eeprom_data);
         
        } // set multi default params - end of block mode == 1

        if (mode == 2) { // set default calibration coefficient
          mode = 10;

          incremental_var = calibration_coeff;
          while(true) {
            delay(100);
            lcd.clear(); 
            lcd.print("Cal = ");
            lcd.print(incremental_var);
            
            if(incremental_var < -1) incremental_var = 0;
            
            if(is_button_pressed) { // if button pressed set default mono mode speed
              calibration_coeff = incremental_var;
              current_calibration = calibration_coeff;
              is_button_pressed = false;
              EEPROM.put(22, calibration_coeff);
              incremental_var = 10;
              break;
            }
          }  
        } // set default calibration coefficient
          
        if (mode == 3) { // show current default params
          mode = 10;
          incremental_var = 0;
          while(true) {
            delay(100);
            lcd.clear(); 
            if (incremental_var == 0) {
              lcd.print("defMono = ");
              lcd.print(to_double(eeprom_data[0]));
              }
            if (incremental_var == 1) {
              lcd.print("defNsteps = ");
              lcd.print(eeprom_data[1]);
              }
            if ((incremental_var > 1) && (incremental_var < eeprom_data[1] + 2))  
              {
                if (eeprom_data[1]> 0) {
                  lcd.clear();
                  value_for_print = to_double(eeprom_data[2 * incremental_var - 2]);
                  print_array_element("v", incremental_var - 1, value_for_print);                  

                  lcd.setCursor(0,1);
                  value_for_print = to_double(eeprom_data[2 * incremental_var -1]);
                  print_array_element("d", incremental_var - 1, value_for_print);
                  delay(100);
                }
                else {
                  lcd.print("No MultiParams");
                  lcd.setCursor(1,1);
                  lcd.print("assigned");
                }
              }
            
            if (incremental_var == eeprom_data[1] + 2) 
              {
              lcd.print("defCal = ");
              lcd.print(calibration_coeff);
              }

              circular_listing(0, eeprom_data[1] + 3);
           
              if(is_button_pressed) { // if button pressed set defaul mono mode speed
                is_button_pressed = false;
                break;
              }
          }
        } // show current default params
        else { // Main screen
          mode = 10;
        } // Main screen
            
       } // default parameters - end of block
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      if(point == 0) { // exit to desktop
        is_menu_active = false;
        point = 10;
        incremental_var = 1;
        }
    
   } // open menu list - end of block
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
   else {          // desktop - start of block
    delay(100);
    lcd.clear(); 
    lcd.print("V = ");
    lcd.print(to_double(current_var));
    lcd.print(" mm/h ");
    lcd.setCursor(4, 1);
    
    if( incremental_var == -1) {
      lcd.print("Menu");
    }
    else {
      lcd.print(to_double(incremental_var));
    }
    
    limit_listing(-1, 200);
    
    lcd.setCursor(9,1);
    lcd.print("NoPull");
    
    if(is_button_pressed) { // if button pressed chose cur_speed or go to menu
      if (incremental_var < 0) {
        is_menu_active = true;
        incremental_var = 0;
      }
      else {
        current_var = incremental_var;
        number_of_steps = 0; // is automatically choosing mode "Mono"
        }
      is_button_pressed = false; 
    }
  } // desktop - end of block
 } // when there is no pulling - end of block

    
/*****************************SOME ANOTHER PARAMETERS**************************************/ 
  
  digitalWrite(INC, HIGH);
  digitalWrite(DECR, HIGH);
  digitalWrite(BUT, HIGH);
  lcd_initialize();
  delay(100);          
}

/************************************** Handlers of interrupts and functions ***********************************/
void encoder() {                           // interrupting on pin 2, decrement, increment
   if(digitalRead(BUT) != 0) {
    if(digitalRead(DECR)) {
      incremental_var--; 
      }
    else {
      incremental_var++; 
      }
     }
   }
  
void button() {                   // interrupting on pin 2, button of encoder
  noInterrupts();
  is_button_pressed = true;
  interrupts();
  }
  
double to_double(int int_value) {                       // change integer value to double value to print in lcd display (for user)
  return (double)int_value / 10;  
  }
  

void timer_interrupt_handler() {                     //handler for timer interrupt                
  if(timer_interrupt_permission) { 
    timer_handler_counter++; 
    if(timer_handler_counter > interrupt_pause) {
      signal_to_step_driver(); 
      timer_handler_counter = 0;  
      }  
    }
  lcd_initialize_counter++; 
  }

void signal_to_step_driver() {               // change pin state which sends impulses to step driver
   if(digitalRead(STEP)) {
    digitalWrite(STEP, LOW); 
    }
   else {
    digitalWrite(STEP, HIGH);
    }
   }

void speed_to_impulses(int vv) {            // conversation of speed from mm / h to a pause between pulses (for interruption)
  if(vv < 1) vv = 1;
  if(vv > 200) vv = 200;
  interrupt_pause = (int)(current_calibration/vv);  // here you can do the calibration changing the numerator
  }


bool is_pulling_allowed() {
  if((digitalRead(termodat) == LOW)&&(length_control == false)) return true;
  else return false;
  }

void dirDOWN() {
  digitalWrite(DIR, HIGH);       // set step driver direction down
  }

void enabled() {
  digitalWrite(enable, HIGH);       // step driver moving enabled
  }
  
void disabled() {
  digitalWrite(enable, LOW);       // step driver moving disabled
  }

void set_default_multi_parameters() {
  number_of_steps = eeprom_data[1];
  for (i = 0; i < number_of_steps; i++) {
    pulling_speeds[i] = eeprom_data[2*i+2];
    pulling_lengths[i] = eeprom_data[2*i+3];
    }
  }

void circular_listing(int first, int last)
    {
    if(incremental_var < first) incremental_var = last;
    if(incremental_var > last)  incremental_var = first;
    }

void limit_listing(int first, int last)
    {
    if(incremental_var < first) incremental_var = first;
    if(incremental_var > last)  incremental_var = last;
    }
    
void lcd_initialize() {
  if(lcd_initialize_counter > 300000) {
    delay(100);
    LiquidCrystal lcd(rs, en, d4, d5, d6, d7);  
    lcd.begin(16, 2); 
    lcd_initialize_counter = 0;
    }
  }
        
void print_array_element(char type[], int idx, double value) { 
  lcd.print(type);
  lcd.print("[");
  lcd.print(idx);
  lcd.print("] = ");
  lcd.print(value);  
  }
