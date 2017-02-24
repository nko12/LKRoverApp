#ifndef LK_HW_H
#define LK_HW_H

class LKHW {
public:
  virtual void setPWMs(double pwms[]);
  virtual void getCount(double time, double pos[]);
};

#endif
