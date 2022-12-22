#ifndef LMS8FEAPP_H
#define LMS8FEAPP_H

#include <wx/app.h>

class ConnectionManager;

class LMS8FE_app : public wxApp
{
public:
    virtual bool OnInit();
};

#endif // LMS8FEAPP_H
