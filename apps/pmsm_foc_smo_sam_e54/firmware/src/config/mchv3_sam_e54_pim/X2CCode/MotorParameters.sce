// (c) 2020 Microchip Technology Inc. and its subsidiaries
// Subject to your compliance with these terms, you may use Microchip software and any derivatives exclusively 
// with Microchip products. Youâ??re responsible for complying with 3rd party license terms applicable to your use 
// of 3rd party software (including open source software) that may accompany Microchip software. 
// SOFTWARE IS â??AS IS.â?? NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
// IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP 
// BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
// KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR 
// THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIPâ??S TOTAL LIABILITY ON ALL CLAIMS RELATED 
// TO THE SOFTWARE WILL NOT EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

if isdef(['motorName'])==%F then
    mprintf("[Warning] - No motorName defined, defining this now.");
    motorName = 'AC300025';
else
    if motorName == 'AC300025'
        if isdef(['Vbus'])==%T then
            if Vbus == 0 then
                Vbus = 325;
            end
        end

        Rs = 1.39;
        Ld = 0.00253;
        Lq = 0.00253;
        Kell = 44.380001;

        n_p = 5;

        nominalSpeed = 3000;

        J_nmradss = 0.912e-6 * 10; // inertia Nm/rad/s/s

        motorRatedCurrent = 4;
        motorStartupCurrent = 0.2;

        boardName == 'dsPICDEM MCHV-3'
        kpCurrent = 0.02;
        kiCurrent = 100;
        kpVelocity = 0.001;
        kiVelocity = 0.0001;
    end
end
