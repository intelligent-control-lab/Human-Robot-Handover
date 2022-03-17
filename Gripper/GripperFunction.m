% This is the file of gripper related functions.
% Copyright (C) 2022  
% 
% Authors:
% Ruixuan Liu: ruixuanl@andrew.cmu.edu
% Rui Chen: ruic3@andrew.cmu.edu
% Yifan Sun: yifansu2@andrew.cmu.edu
% Changliu Liu : cliu6@andrew.cmu.edu
% 
% This program is free software; you can redistribute it and/or
% modify it under the terms of the GNU General Public License
% as published by the Free Software Foundation; either version 2
% of the License, or (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program; if not, write to the Free Software
% Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

function gripper = GripperFunction
    gripper.buildInputDic = @buildInputDic;
    gripper.buildOutputDic = @buildOutputDic;
    
    gripper.connect = @connect;
    
    gripper.reset = @reset;
    gripper.activate = @activate;
    gripper.close = @close;
    gripper.open = @open;
    gripper.goto = @goto;
    
    gripper.gotoFinger = @gotoFinger;
    gripper.gotoScissor = @gotoScissor;
    
    gripper.printInputInfo = @printInputInfo;
    gripper.readInput = @readInput;
    gripper.printOutputInfo = @printOutputInfo;
    gripper.readOutput = @readOutput;
end

function inputDic = buildInputDic()
    gACT(1) = " Gripper reset." ;
    gACT(2) = " Gripper activation.";
    gMOD(1) = " Basic mode.";
    gMOD(2) = " Pinch mode.";
    gMOD(3) = " Wide mode.";
    gMOD(4) = " Scissor mode.";
    gGTO(1) = " Stopped (or performing activation / grasping mode change / automatic release).";
    gGTO(2) = " Go to Position Request.";
    gIMC(1) = " Gripper is in reset (or automatic release) state. See Fault status if Gripper is activated.";
    gIMC(2) = " Activation is in progress.";
    gIMC(3) = " Mode change is in progress.";
    gIMC(4) = " Activation and Mode change are complete.";
    gSTA(1) = " Gripper is in motion towards requested position (only meaningful if gGTO = 1).";
    gSTA(2) = " Gripper is stopped. One or two fingers stopped before requested position.";
    gSTA(3) = " Gripper is stopped. All fingers stopped before requested position.";
    gSTA(4) = " Gripper is stopped. All fingers reached requested position.";
    
    %Byte 1 Object Status
    gDTA(1) = " Finger A is in motion (only meaningful if gGTO = 1).";
    gDTA(2) = " Finger A has stopped due to a contact while opening.";
    gDTA(3) = " Finger A has stopped due to a contact while closing.";
    gDTA(4) = " Finger A is at the requested position.";
    gDTB(1) = " Finger B is in motion (only meaningful if gGTO = 1).";
    gDTB(2) = " Finger B has stopped due to a contact while opening.";
    gDTB(3) = " Finger B has stopped due to a contact while closing.";
    gDTB(4) = " Finger B is at the requested position.";
    gDTC(1) = " Finger C is in motion (only meaningful if gGTO = 1).";
    gDTC(2) = " Finger C has stopped due to a contact while opening.";
    gDTC(3) = " Finger C has stopped due to a contact while closing.";
    gDTC(4) = " Finger C is at the requested position.";
    gDTS(1) = " Scissor is in motion (only meaningful if gGTO = 1).";
    gDTS(2) = " Scissor has stopped due to a contact while opening.";
    gDTS(3) = " Scissor has stopped due to a contact while closing.";
    gDTS(4) = " Scissor is at the requested position.";
    
    %Byte 2 Fault Status
    gFLT(1) = " No fault (fault LED off)";
    gFLT(6) = " Priority faults (LED is blue). Action delayed, activation (reactivation) must be completed prior to renewed action.";
    gFLT(7) = " Priority faults (LED is blue). Action delayed, mode change must be completed prior to continuing action.";
    gFLT(8) = " Priority faults (LED is blue). The activation bit must be set prior to action.";
    gFLT(10) = " Minor faults (LED continuous red). The communication chip is not ready (may be booting).";
    gFLT(11) = " Minor faults (LED continuous red). Changing mode fault, interference detected on Scissor (for less than 20 sec).";
    gFLT(12) = " Minor faults (LED continuous red). Automatic release in progress.";
    gFLT(14) = " Major faults (LED blinking red/blue) - Reset is required. Activation fault, verify that no interference or other error occurred.";
    gFLT(15) = " Major faults (LED blinking red/blue) - Reset is required. Changing mode fault, interference detected on Scissor (for more than 20 sec).";
    gFLT(16) = " Major faults (LED blinking red/blue) - Reset is required. Automatic release completed. Reset and activation is required.";
    
    
    
    
    for i = 1:1:256
        %Byte 3 Finger A Position Request Echo
        gPRA(i) = ["Echo of the requested position for the Gripper(Finger A): "+ num2str((i - 1)) + "/255"];
        %Byte 4 Finger A Position
        gPOA(i) = ["Actual position of the Gripper(Finger A) obtained via the encoders: "+ num2str((i - 1)) + "/255"];
        %Byte 5 Finger A Current
        gCUA(i) = ["The current is read instantaneously from the motor drive(Finger A), approximate current: "+ num2str((i - 1)) + "/255"];
        %Byte 6 Finger B Position Request Echo
        gPRB(i) = ["Echo of the requested position for Finger B: "+ num2str((i - 1)) + "/255"];
        %Byte 7 Finger B Position
        gPOB(i) = ["Actual position of Finger B obtained via the encoders: "+ num2str((i - 1)) + "/255"];
        %Byte 8 Finger B Current
        gCUB(i) = ["The current is read instantaneously from the motor drive(Finger B), approximate current: "+ num2str((i - 1)) + "/255"];
        %Byte 9 Finger C Position Request Echo
        gPRC(i) = ["Echo of the requested position for Finger C: "+ num2str((i - 1)) + "/255"];
        %Byte 10 Finger C Position
        gPOC(i) = ["Actual position of Finger C obtained via the encoders: "+ num2str((i - 1)) + "/255"];
        %Byte 11 Finger C Current
        gCUC(i) = ["The current is read instantaneously from the motor drive(Finger C), approximate current: "+ num2str((i - 1)) + "/255"];
        %Byte 12 Scissor Position Request Echo
        gPRS(i) = ["Echo of the requested position for Scissor: "+ num2str((i - 1)) + "/255"];
        %Byte 13 Scissor Position
        gPOS(i) = ["Actual position of Scissor obtained via the encoders: "+ num2str((i - 1)) + "/255"];
        %Byte 14 Scissor Current
        gCUS(i) = ["The current is read instantaneously from the motor drive(Scissor), approximate current: "+ num2str((i - 1)) + "/255"];
    end

    inputDic.gACT = gACT;
    inputDic.gMOD = gMOD;
    inputDic.gGTO = gGTO;
    inputDic.gIMC = gIMC;
    inputDic.gSTA = gSTA;
    inputDic.gDTA = gDTA;
    inputDic.gDTB = gDTB;
    inputDic.gDTC = gDTC;
    inputDic.gDTS = gDTS;

    inputDic.gFLT = gFLT;

    inputDic.gPRA = gPRA;
    inputDic.gPOA = gPOA;
    inputDic.gCUA = gCUA;
    inputDic.gPRB = gPRB;
    inputDic.gPOB = gPOB;
    inputDic.gCUB = gCUB;
    inputDic.gPRC = gPRC;
    inputDic.gPOC = gPOC;
    inputDic.gCUC = gCUC;
    inputDic.gPRS = gPRS;
    inputDic.gPOS = gPOS;
    inputDic.gCUS = gCUS;
end

function outputDic = buildOutputDic()   
    %Byte 0 Action Request
    rACT(1) = " Deactivate Gripper." ;
    rACT(2) = " Activate Gripper (must stay on after activation routine is completed).";
    rMOD(1) = " Go to Basic Mode.";
    rMOD(2) = " Go to Pinch mode.";
    rMOD(3) = " Go to Wide mode.";
    rMOD(4) = " Go to Scissor mode.";
    rGTO(1) = " Stop.";
    rGTO(2) = " Go to Position Request.";
    rATR(1) = " Normal.";
    rATR(2) = " Emergency auto-release.";
    
    %Byte 1 Gripper Option 1
    rICF(1) = " Normal.";
    rICF(2) = " Enable Individual Control of Fingers A, B and C.";
    rICS(1) = " Normal.";
    rICS(2) = " Enable Individual Control of Scissor. Disable Mode Selection.";
    
    %Byte 2 Gripper Option 2 Reserved
   
    for i = 1:1:256
        %Byte 3 Position Request(Finger A in individual mode)
        rPRA(i) = ["Position of the Gripper(Finger A): "+ num2str((i - 1)) + "/255"];
        %Byte 4 Speed(Finger A in individual mode)
        rSPA(i) = ["Speed of the Gripper(Finger A): "+ num2str((i - 1)) + "/255"];
        %Byte 5 Force(Finger A in individual mode)
        rFRA(i) = ["Force of the Gripper(Finger A): "+ num2str((i - 1)) + "/255"];
        %Byte 6 Finger B Position
        rPRB(i) = ["Position of Finger B: "+ num2str((i - 1)) + "/255"];
        %Byte 7 Finger B Speed
        rSPB(i) = ["Speed of Finger B: "+ num2str((i - 1)) + "/255"];
        %Byte 8 Finger B Force
        rFRB(i) = ["Force of Finger B: "+ num2str((i - 1)) + "/255"];
        %Byte 9 Finger C Position
        rPRC(i) = ["Position of Finger C: "+ num2str((i - 1)) + "/255"];
        %Byte 10 Finger C Speed
        rSPC(i) = ["Speed of Finger C: "+ num2str((i - 1)) + "/255"];
        %Byte 11 Finger C Force
        rFRC(i) = ["Force of Finger C: "+ num2str((i - 1)) + "/255"];
        %Byte 12 Scissor Position
        rPRS(i) = ["Position of Scissor: "+ num2str((i - 1)) + "/255"];
        %Byte 13 Scissor Speed
        rSPS(i) = ["Speed of Scissor: "+ num2str((i - 1)) + "/255"];
        %Byte 14 Scissor Force
        rFRS(i) = ["Force of Scissor: "+ num2str((i - 1)) + "/255"];
    end

    outputDic.rACT = rACT;
    outputDic.rMOD = rMOD;
    outputDic.rGTO = rGTO;
    outputDic.rATR = rATR;
    outputDic.rICF = rICF;
    outputDic.rICS = rICS;
    
    outputDic.rPRA = rPRA;
    outputDic.rSPA = rSPA;
    outputDic.rFRA = rFRA;
    outputDic.rPRB = rPRB;
    outputDic.rSPB = rSPB;
    outputDic.rFRB = rFRB;
    outputDic.rPRC = rPRC;
    outputDic.rSPC = rSPC;
    outputDic.rFRC = rFRC;
    outputDic.rPRS = rPRS;
    outputDic.rPRS = rPRS;
    outputDic.rFRS = rFRS;
end

function input = readInput(m)
    inputregs = read(m,'inputregs',1,8);
    inputregs = dec2bin(inputregs, 16);
    
    %Byte 0 Gripper Status
    input.gSTA = bin2dec(inputregs(1,1:2));
    input.gIMC = bin2dec(inputregs(1,3:4));
    input.gGTO = bin2dec(inputregs(1,5));
    input.gMOD = bin2dec(inputregs(1,6:7));
    input.gACT = bin2dec(inputregs(1,8));
    %Byte 1 Object Status
    input.gDTS = bin2dec(inputregs(1,9:10));
    input.gDTC = bin2dec(inputregs(1,11:12));
    input.gDTB = bin2dec(inputregs(1,13:14));
    input.gDTA = bin2dec(inputregs(1,15:16));
    %****************************************
    %Byte 2 Fault Status
    input.gFLT = bin2dec(inputregs(2,5:8));
    %Byte 3 Finger A Position Request Echo
    input.gPRA = bin2dec(inputregs(2,9:16));
    %****************************************
    %Byte 4 Finger A Position
    input.gPOA = bin2dec(inputregs(3,1:8));
    %Byte 5 Finger A Current
    input.gCUA = bin2dec(inputregs(3,9:16));
    %****************************************
    %Byte 6 Finger B Position Request Echo
    input.gPRB = bin2dec(inputregs(4,1:8));
    %Byte 7 Finger B Position
    input.gPOB = bin2dec(inputregs(4,9:16));
    %****************************************
    %Byte 8 Finger B Current
    input.gCUB = bin2dec(inputregs(5,1:8));
    %Byte 9 Finger C Position Request Echo
    input.gPRC = bin2dec(inputregs(5,9:16));
    %****************************************
    %Byte 10 Finger C Position
    input.gPOC = bin2dec(inputregs(6,1:8));
    %Byte 11 Finger C Current
    input.gCUC = bin2dec(inputregs(6,9:16));
    %****************************************
    %Byte 12 Scissor Position Request Echo
    input.gPRS = bin2dec(inputregs(7,1:8));
    %Byte 13 Scissor Position
    input.gPOS = bin2dec(inputregs(7,9:16));
    %****************************************
    %Byte 14 Scissor Current
    input.gCUS = bin2dec(inputregs(8,1:8));
end

function output = readOutput(m)
	outputregs = read(m,'holdingregs',1,8);
    outputregs = dec2bin(outputregs, 16);
    
    %Byte 0 Action Request
    output.rATR = bin2dec(outputregs(1,4));
    output.rGTO = bin2dec(outputregs(1,5));
    output.rMOD = bin2dec(outputregs(1,6:7));
    output.rACT = bin2dec(outputregs(1,8));
    %Byte 1 Gripper Option 1
    output.rICS = bin2dec(outputregs(1,13));
    output.rICF = bin2dec(outputregs(1,14));
    %****************************************
    %Byte 2 Gripper Option 2
    %Byte 3 Finger A Position Request
    output.rPRA = bin2dec(outputregs(2,9:16));
    %****************************************
    %Byte 4 Finger A Speed
    output.rSPA = bin2dec(outputregs(3,1:8));
    %Byte 5 Finger A Force
    output.rFRA = bin2dec(outputregs(3,9:16));
    %****************************************
    %Byte 6 Finger B Position Request
    output.rPRB = bin2dec(outputregs(4,1:8));
    %Byte 7 Finger B Speed
    output.rSPB = bin2dec(outputregs(4,9:16));
    %****************************************
    %Byte 8 Finger B Force
    output.rFRB = bin2dec(outputregs(5,1:8));
    %Byte 9 Finger C Position Request
    output.rPRC = bin2dec(outputregs(5,9:16));
    %****************************************
    %Byte 10 Finger C Speed
    output.rSPC = bin2dec(outputregs(6,1:8));
    %Byte 11 Finger C Force
    output.rFRC = bin2dec(outputregs(6,9:16));
    %****************************************
    %Byte 12 Scissor Position Request
    output.rPRS = bin2dec(outputregs(7,1:8));
    %Byte 13 Scissor Speed
    output.rSPS = bin2dec(outputregs(7,9:16));
    %****************************************
    %Byte 14 Scissor Force
    output.gFRS = bin2dec(outputregs(8,1:8));
end

function m = connect(type, IP, port)
    m = modbus(type, IP, port);
%     pause(0.1)
end

function reset(m)
    write(m,'holdingregs',1,hex2dec(["0" "0" "0"]))
end

function activate(m)
    write(m,'holdingregs',1,hex2dec(["0104" "0" "0"]));
    info = "Activating";
    tic
    while 1
        input = readInput(m);      
        t = toc;
        if input.gIMC == 3
            info = "Activation completed"
            break;
        elseif t > 20
            info ="Activation failed. Time out."
            break;
        end
    end
    readInput(m);
    readOutput(m);
end



function goto(m, mode, check, position, speed, force)
%position, speed and force are in ranger from 0 to 255
    info = "*******************Gripper is moving*********************"
    %mode = "WIDE"
    if mode == "BASIC"
        mode16 = "09";
    elseif mode == "PINCH"
        mode16 = "0B";
    elseif mode == "WIDE"
        mode16 = "0D";
    elseif mode == "SCISSOR"
        mode16 = "0F";
    end
    r1 = strcat(mode16, "00");
    
    if position > 255
        info = "maximum position is 255"
    elseif speed > 255
        info = "maximum speed is 255"
    elseif force > 255
        info = "maximum force is 255"
    else
        position16 = dec2hex(position, 2);
        speed16 = dec2hex(speed, 2);
        force16 = dec2hex(force, 2);
        r2 = strcat('00', position16);
        r3 = strcat(speed16, force16);
        write(m,'holdingregs',1,hex2dec([r1 r2 r3]))
       
        tic
        while check == 1
            pause(0.1)
            input = readInput(m);      
            t = toc;
            if input.gSTA == 3
                info = strcat("Gripper stopped. Reached requested position: ", num2str(position))
                break;
            elseif input.gSTA == 1 || input.gSTA == 2
                info = "Gripper stopped. Object detected"
                break;
            elseif t > 20
                info ="Gripper stopped before reaching requested position. Time out."
                break;
            end
        end
        input = readInput(m)
        output = readOutput(m)
        
    end
    
end

function gotoScissor(m, withFinger, check, positionA, positionB, positionC, positionS, speedA, speedB, speedC, speedS, forceA, forceB, forceC, forceS)
%position, speed and force are in ranger from 0 to 255
    info = "*******************Gripper is moving*********************"
    info = "When ICS is on, mode will be ignored."
    if withFinger == 1
        mode16 = "0C";
    elseif withFinger == 0
        mode16 = "08";
    else
        info = "ICS will disable model selection. Please choose from 1 or 0. "
    end
    r1 = strcat("09", mode16);
    
    if positionA > 255 || positionB > 255 || positionC > 255 || positionS > 255
        info = "maximum position is 255"
    elseif speedA > 255 || speedB > 255 || speedC > 255 || speedS > 255
        info = "maximum speed is 255"
    elseif forceA > 255 || forceB > 255 || forceC > 255 || forceS > 255
        info = "maximum force is 255"
    else
        positionA16 = dec2hex(positionA, 2);
        speedA16 = dec2hex(speedA, 2);
        forceA16 = dec2hex(forceA, 2);
        positionB16 = dec2hex(positionB, 2);
        speedB16 = dec2hex(speedB, 2);
        forceB16 = dec2hex(forceB, 2);
        positionC16 = dec2hex(positionC, 2);
        speedC16 = dec2hex(speedC, 2);
        forceC16 = dec2hex(forceC, 2);
        positionS16 = dec2hex(positionS, 2);
        speedS16 = dec2hex(speedS, 2);
        forceS16 = dec2hex(forceS, 2);
        
        
        r2 = strcat('00', positionA16);
        r3 = strcat(speedA16, forceA16);
        r4 = strcat(positionB16, speedB16);
        r5 = strcat(forceB16, positionC16);
        r6 = strcat(speedC16, forceC16);
        r7 = strcat(positionS16, speedS16);
        r8 = strcat(forceS16, '00');
        
        write(m,'holdingregs',1,hex2dec([r1 r2 r3 r4 r5 r6 r7 r8]))
       
        tic
        while check == 1
            pause(0.1)
            input = readInput(m);      
            t = toc;
            if input.gSTA == 3
                info = strcat("Gripper stopped. Reached requested position: ", num2str(positionA)," ", num2str(positionB)," ", num2str(positionC)," ",num2str(positionS))
                break;
            elseif input.gSTA == 1 || input.gSTA == 2
                info = "Gripper stopped. Object detected"
                break;
            elseif t > 20
                info ="Gripper stopped before reaching requested position. Time out."
                break;
            end
        end
        input = readInput(m)
        output = readOutput(m)
        
    end
    
end

function gotoFinger(m, mode, check, positionA, positionB, positionC, speedA, speedB, speedC, forceA, forceB, forceC)
%position, speed and force are in ranger from 0 to 255
    info = "*******************Gripper is moving*********************"
    %mode = "WIDE"
    if mode == "BASIC"
        mode16 = "09";
    elseif mode == "PINCH"
        mode16 = "0B";
    elseif mode == "WIDE"
        mode16 = "0D";
    elseif mode == "SCISSOR"
        info = "In Scissor mode the gripper can not move by each finger."
    end
    r1 = strcat(mode16, "04");
    
    if positionA > 255 || positionB > 255 || positionC > 255
        info = "maximum position is 255"
    elseif speedA > 255 || speedB > 255 || speedC > 255
        info = "maximum speed is 255"
    elseif forceA > 255 || forceB > 255 || forceC > 255
        info = "maximum force is 255"
    else
        positionA16 = dec2hex(positionA, 2);
        speedA16 = dec2hex(speedA, 2);
        forceA16 = dec2hex(forceA, 2);
        positionB16 = dec2hex(positionB, 2);
        speedB16 = dec2hex(speedB, 2);
        forceB16 = dec2hex(forceB, 2);
        positionC16 = dec2hex(positionC, 2);
        speedC16 = dec2hex(speedC, 2);
        forceC16 = dec2hex(forceC, 2);
        
        r2 = strcat('00', positionA16);
        r3 = strcat(speedA16, forceA16);
        r4 = strcat(positionB16, speedB16);
        r5 = strcat(forceB16, positionC16);
        r6 = strcat(speedC16, forceC16);
        write(m,'holdingregs',1,hex2dec([r1 r2 r3 r4 r5 r6]))
       
        tic
        while check == 1
            pause(0.1)
            input = readInput(m);      
            t = toc;
            if input.gSTA == 3
                info = strcat("Gripper stopped. Reached requested position: ", num2str(positionA)," ", num2str(positionB)," ", num2str(positionC))
                break;
            elseif input.gSTA == 1 || input.gSTA == 2
                info = "Gripper stopped. Object detected"
                break;
            elseif t > 20
                info ="Gripper stopped before reaching requested position. Time out."
                break;
            end
        end
%         input = readInput(m)
%         output = readOutput(m)
        
    end
    
end

function close(m)
    goto(m, "BASIC", 0,  255, 200, 50)
end

function open(m)
    goto(m, "BASIC", 0, 0, 200, 50)
end

function printInputInfo(m, inputDic)
    input = readInput(m);
    fileds = fieldnames(input);
    for i=1:size(fileds, 1)
        k = fileds(i);
        key = k{1};
        value = input.(key);
        info =strcat(key,' = ',num2str(value),': ', inputDic.(key)(value + 1))
    end
end

function printOutputInfo(m, outputDic)
    output = readOutput(m);
    fileds = fieldnames(output);
    for i=1:size(fileds, 1)
        k = fileds(i);
        key = k{1};
        value = output.(key);
        info =strcat(key,' = ',num2str(value),': ', outputDic.(key)(value + 1))
    end
end



