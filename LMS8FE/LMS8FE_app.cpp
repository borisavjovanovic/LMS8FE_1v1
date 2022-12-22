#ifdef WX_PRECOMP
#include "wx_pch.h"
#endif

#ifdef __BORLANDC__
#pragma hdrstop
#endif //__BORLANDC__

#include "LMS8FE_app.h"
#include "LMS8FE_wxgui.h"
#include <wx/time.h>
#include <wx/splash.h>
#include <wx/icon.h>
#include <iostream>

// #ifndef NDEBUG
// IMPLEMENT_APP_CONSOLE(hamFEsuiteApp);
// #else
// IMPLEMENT_APP(hamFEsuiteApp);
// #endif

IMPLEMENT_APP(LMS8FE_app);

#include "resources/splash.h"
#include "resources/LMS_ICO.xpm"

bool LMS8FE_app::OnInit()
{
	wxInitAllImageHandlers();
	wxBitmap splashBitmap = wxBITMAP_PNG_FROM_DATA(splash);
	wxSplashScreen *splash = new wxSplashScreen(splashBitmap,
												wxSPLASH_CENTRE_ON_SCREEN,
												6000, NULL, -1, wxDefaultPosition, wxDefaultSize,
												wxSIMPLE_BORDER | wxSTAY_ON_TOP);
	wxYield(); // linux needs this to load splash image
	LMS8FE_wxgui *frame = new LMS8FE_wxgui(0L);
#ifndef NDEBUG
	wxLongLong t1 = wxGetUTCTimeMillis();
	std::cout << "Create time " << (wxGetUTCTimeMillis() - t1).ToString() << " ms\n";
#endif
	splash->Destroy();
	frame->SetIcon(wxICON(LMS_ICO));
	frame->Show();
	return true;
}
