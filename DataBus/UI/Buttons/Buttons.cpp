#include "Buttons.h"
#include "PinDetect.h"

PinDetect nextButton(p14);
PinDetect selectButton(p16);            // Input selectButton
PinDetect prevButton(p15);

Buttons::Buttons(void): which(0), pressed(false)
{
}

void Buttons::init()
{

    // Set up button (plugs into two GPIOs, active low
    selectButton.mode(PullUp);
    selectButton.setSamplesTillAssert(50);
    selectButton.setAssertValue(0); // active low logic
    selectButton.setSampleFrequency(50); // us
    selectButton.attach_asserted( this, &Buttons::selectPressed );
    
    nextButton.mode(PullUp);
    nextButton.setSamplesTillAssert(50);
    nextButton.setAssertValue(0); // active low logic
    nextButton.setSampleFrequency(50); // us
    nextButton.attach_asserted( this, &Buttons::nextPressed );

    prevButton.mode(PullUp);
    prevButton.setSamplesTillAssert(50);
    prevButton.setAssertValue(0); // active low logic
    prevButton.setSampleFrequency(50); // us
    prevButton.attach_asserted( this, &Buttons::prevPressed );
}

void Buttons::nextPressed() 
{
    pressed = true;
    which = NEXT_BUTTON;
}

void Buttons::prevPressed() 
{
    pressed = true;
    which = PREV_BUTTON;
}

void Buttons::selectPressed()
{
    pressed = true;
    which = SELECT_BUTTON;
}
