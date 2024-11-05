function Estop = monitorEstop(comPort)
    % Function to monitor Estop value from serial communication
    
    % Validate input argument
    if nargin < 1
        comPort = 'COM5';  % Default COM port if not provided
    end
    
    % Initialize the Arduino serial object
    arduinoObj = serial(comPort, 'BaudRate', 9600);
    
    % Open the serial port
    fopen(arduinoObj);
    
    % Pause to allow the connection to stabilize
    pause(2);
    
    % Initialize the Estop variable
    Estop = int32(0);  % Ensure Estop is an integer
    
    try
        % Read data once from the serial port
        data = fscanf(arduinoObj);
        
        % Convert the data to a numeric value (assuming it's numeric)
        Estop = int32(str2double(data(1, 1)));  % Convert to integer
        
        % Display the Estop value
        disp(['Estop Value: ', num2str(Estop)]);
        
    catch ME
        % Handle any errors and close the serial port
        disp('Error occurred: ');
        disp(ME.message);
    end
    
    % Close the serial port after use
    fclose(arduinoObj);
    delete(arduinoObj);  % Clean up the serial object
    
    % Return the final value of Estop as an integer
    return;
end
