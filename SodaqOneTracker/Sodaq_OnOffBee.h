#ifndef SODAQ_ONOFFBEE_H_
#define SODAQ_ONOFFBEE_H_
/*
 */

/*!
 * \brief This class is used to switch on or off a (SODAQ) Bee device.
 *
 * It's a pure virtual class, so you'll have to implement a specialized
 * class.
 */
class Sodaq_OnOffBee
{
public:
    virtual ~Sodaq_OnOffBee() {}
    virtual void on() = 0;
    virtual void off() = 0;
    virtual bool isOn() = 0;
};

#endif /* SODAQ_ONOFFBEE_H_ */
