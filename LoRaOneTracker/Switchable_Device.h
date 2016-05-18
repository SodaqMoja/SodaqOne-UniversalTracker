#ifndef SWITCHABLE_DEVICE_H
#define SWITCHABLE_DEVICE_H

typedef void(*voidFuncPtr)(void); 

class SwitchableDevice {
private:
  voidFuncPtr _onMethod;
  voidFuncPtr _offMethod;

public:
  SwitchableDevice();

  void setOnMethod(voidFuncPtr onMethod);
  void setOffMethod(voidFuncPtr offMethod);

  void setSwitchMethods(voidFuncPtr onMethod, voidFuncPtr offMethod);
  void clearSwitchMethods();

  void on();
  void off();
};

#endif // SWITCHABLE_DEVICE_H
