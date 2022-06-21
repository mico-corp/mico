                                                                                
                                                                                
                                                                                
                                   %%%%%                                        
                                    %%%%%%%%                                    
                                   %%%%   %%%%                                  
                             %%%%%%%%       %%%%%%%%                            
                         %%%%%%  %%%               %%%%%                        
                      %%%%       %%                    %%%%                     
                    %%%%                                 %%%%                   
                  %%%%                                     %%%%                 
        %%%%%%%%%%%%                                         %%%%%%%%%%%        
      %%%%      %%%       %%%%%%%%%%%%     %%%%%%%%%%%%        %%      %%%%     
     %%%    %%%%%%     %%%%          %%%%%%%          %%%%     %%%%%%    %%%    
     %%% %%%% %%%     %%%                               %%%     %%% %%%  %%%    
     %%% %%% %%%     %%%     %%%%%            %%%%%%     %%%     %%% %%% %%%    
     %%%  %%%%%%      %%     %%%%%             %%%%      %%      %%%%%%  %%%    
      %%%%   %%%       %%%                             %%%       %%%   %%%      
         %%%%%%%        %%%%%                       %%%%%        %%%%%%%        
              %%           %%%%%%                %%%%%           %%             
              %%%            %%%     %%   %%     %%%            %%%             
               %%            %%%                 %%%            %%              
               %%%%%%%      %%% %%%           %%% %%       %%%%%%%              
                 %%%%%       %%% %%%%%%%%%%%%%%%%%%%       %%%%%%               
                   %%%%%%%     %%               %%%    %%%%%%%                  
                    %%%%%%%%%  %%%%%         %%%%%  %%%%%%%%%                   
                        %%%%%%    %%%%%%%%%%%%%    %%%%%%                       
                            %%%%%               %%%%%                           
                               %%%%%%%%%%%%%%%%%%                               
                                                                                
                                                                                


# CHANGES LOG!

## Version 2.0.0-alpha

Released in June 2022. For more information about the previous status, go to previous releases. From now on, everything will be written here too!

### New Features

* Simulation and Robotics blocks have been restored, including a PID block and 1st and 2nd order system simulation.
* Super cool new feature, pressing ctrl+D duplicates blocks!
* The way flow is coded has been improbed. Dataflows are not exposed anymore. Callbacks registered have explicitly the input arguments they require and when doing the registration, these are compare against the inputs configured in the Policy (being type safe then). This makes the code easier to understand.
* Inputs can be configured as consumable or not.
* The Plotter has been improved a lot! Now the data displayed is smoother and fits better to real data.


### Known Bugs