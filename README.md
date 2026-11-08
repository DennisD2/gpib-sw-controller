# GPIB software only controller code
GPIB software only controller code for AVR micro controllers.

### Code Description
See doc/html/index.html

### List of good test commands:

#### Tek 2432a
```bash
INIT
MEASUREMENT WINDOW:ON
CURSOR FUNCTION: TIME,TARGET:CH1,UNITS:TIME:BASE
CURSOR TPOS:ONE:200,TPOS:TWO:500
message 5:"dennis testet hier auch"
ID?
RUN?
CH1?
wavfrm?
curve?
wfmpre?
data encdg:ascii
```

#### Tek 1241
```bash
id?
dt?
refmem?
```