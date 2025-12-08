function [g1,g2,g3,g4,g5,g6] = my_PWM(sector,u,t,Ts,Vdc)
   if u >= Vdc  %u >= 0
       u = Vdc;
   end

   t1 = [u/Vdc*Ts,Ts];%use this method to simulate PWM pulse output
   g_num = [1 0];
   for j = 1:2
       if t < t1(j)
           break;
       end
   end
   g = g_num(j);
   switch sector
       case 1
           pulses = [g,0,0,1,0,0];
       case 2
           pulses = [g,0,0,0,0,1];
       case 3
           pulses = [0,0,g,0,0,1];
       case 4
           pulses = [0,1,g,0,0,0];
       case 5
           pulses = [0,1,0,0,g,0];
       case 6
           pulses = [0,0,0,1,g,0];
       otherwise
           pulses = [0,0,0,0,0,0];
   end
g1 = pulses(1);
g2 = pulses(2);
g3 = pulses(3);
g4 = pulses(4);
g5 = pulses(5);
g6 = pulses(6);


    
