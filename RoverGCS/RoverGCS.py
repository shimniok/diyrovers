#!/usr/bin/python

# DIY Rovers Ground Control Station
#


# -*- coding: <<encoding>> -*-
#-------------------------------------------------------------------------------
#   <<project>>
#
#-------------------------------------------------------------------------------

import wxversion

wxversion.select("2.8")
import wx
import wx.html
import sys
from math import pi

aboutText = """<p>Sorry, there is no information about this program. It is
running on version %(wxpy)s of <b>wxPython</b> and %(python)s of <b>Python</b>.
See <a href="http://wiki.wxpython.org">wxPython Wiki</a></p>"""


class HtmlWindow(wx.html.HtmlWindow):
    def __init__(self, parent, id, size=(600, 400)):
        wx.html.HtmlWindow.__init__(self, parent, id, size=size)
        if "gtk2" in wx.PlatformInfo:
            self.SetStandardFonts()

    def OnLinkClicked(self, link):
        wx.LaunchDefaultBrowser(link.GetHref())


class AboutBox(wx.Dialog):
    def __init__(self):
        wx.Dialog.__init__(self, None, -1, "About <<project>>",
                           style=wx.DEFAULT_DIALOG_STYLE | wx.THICK_FRAME | wx.RESIZE_BORDER |
                           wx.TAB_TRAVERSAL)
        hwin = HtmlWindow(self, -1, size=(400, 200))
        vers = {}
        vers["python"] = sys.version.split()[0]
        vers["wxpy"] = wx.VERSION_STRING
        hwin.SetPage(aboutText % vers)
        btn = hwin.FindWindowById(wx.ID_OK)
        irep = hwin.GetInternalRepresentation()
        hwin.SetSize((irep.GetWidth() + 25, irep.GetHeight() + 10))
        self.SetClientSize(hwin.GetSize())
        self.CentreOnParent(wx.BOTH)
        self.SetFocus()


#class NeedlePanel():
#    def __init__(self, imagepath, range=(0.0,1.0), anglerange=(-160.0, 160.0)):
#    def update(self, value):
#    def angle(self, angle):
# grab image
# rotate image
# refresh


class GaugePanel(wx.Panel):
    def __init__(self, parent, imagepath, width=100, height=100):
        wx.Panel.__init__(self, parent)
        self.image = wx.Image(imagepath)
        self.width = width
        self.height = height
        self.image = self.image.Scale(width, height)
        wx.StaticBitmap(self, -1, wx.BitmapFromImage(self.image))
        self.needle = {}

    def AddNeedle(self, imagepath, name=0):
        self.needle[name] = wx.Image(imagepath)
        self.needle[name] = self.needle[name].Scale(self.width, self.height)
        self.SetAngle(angle=0, name=name)

    def SetAngle(self, angle, name=0):
        n = self.needle[name]
        width = n.GetWidth()
        height = n.GetHeight()
        n = n.Rotate(-angle*pi/180.0, None, False, None)
        dw = n.GetWidth() - width
        dh = n.GetHeight() - height
        rect = wx.Rect(dw/2, dh/2, width-dw/2, height-dh/2)
        n = n.GetSubImage(rect)
        wx.StaticBitmap(self, -1, wx.BitmapFromImage(self.image))
        wx.StaticBitmap(self, -1, wx.BitmapFromImage(n))
        self.Refresh()

class Frame(wx.Frame):
    def __init__(self, title):
        wx.Frame.__init__(self, None, title=title, pos=(150, 150), size=(955, 400))
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        menubar = wx.MenuBar()
        menu = wx.Menu()
        m_exit = menu.Append(wx.ID_EXIT, "E&xit\tAlt-X", "Close window and exit program.")
        self.Bind(wx.EVT_MENU, self.OnClose, m_exit)
        menubar.Append(menu, "&File")
        menu = wx.Menu()
        m_about = menu.Append(wx.ID_ABOUT, "&About", "Information about this program")
        self.Bind(wx.EVT_MENU, self.OnAbout, m_about)
        menubar.Append(menu, "&Help")
        self.SetMenuBar(menubar)

        self.statusbar = self.CreateStatusBar()

        panel = wx.Panel(self)
        # bmp = wx.Image("resources/maple.jpg", wx.BITMAP_TYPE_ANY).ConvertToBitmap()
        # self.bitmap1 = wx.StaticBitmap(panel, -1, bmp, (0, 0))
        sizer = wx.GridBagSizer(5, 5)

        speedo_panel = GaugePanel(panel, "resources/speedometer1.png", 300, 300)
        speedo_panel.SetBackgroundColour("#990000")
        speedo_panel.AddNeedle("resources/speedometerneedle1.png")
        # speedo_panel.SetAngle(100)
        # speedo_panel.SetAngle(10)
        sizer.Add(speedo_panel, pos=(0, 0), span=(2, 2), flag=wx.EXPAND|wx.LEFT|wx.RIGHT, border=5)

        volt_panel = GaugePanel(panel, "resources/voltmeter1.png", 150, 150)
        volt_panel.AddNeedle("resources/voltmeterneedle1.png")
        sizer.Add(volt_panel, pos=(0, 2), span=(1, 1), flag=wx.EXPAND|wx.LEFT|wx.RIGHT, border=5)

        ammeter_panel = GaugePanel(panel, "resources/ammeter2.gif", 150, 150)
        ammeter_panel.AddNeedle("resources/ammeterneedle2.gif")
        sizer.Add(ammeter_panel, pos=(0, 3), span=(1, 1), flag=wx.EXPAND|wx.LEFT|wx.RIGHT, border=5)

        fuel_panel = GaugePanel(panel, "resources/fuel1.png", 150, 150)
        fuel_panel.AddNeedle("resources/fuelneedle1.png")
        sizer.Add(fuel_panel, pos=(1, 2), span=(1, 1), flag=wx.EXPAND|wx.LEFT|wx.RIGHT, border=5)

        clock_panel = GaugePanel(panel, "resources/clock.png", 150, 150)
        clock_panel.AddNeedle("resources/clockhour.png", "hour")
        clock_panel.AddNeedle("resources/clockminute.png", "minute")
        clock_panel.AddNeedle("resources/clocksecond.png", "second")
        sizer.Add(clock_panel, pos=(1, 3), span=(1, 1), flag=wx.EXPAND|wx.LEFT|wx.RIGHT, border=5)

        compass_panel = GaugePanel(panel, "resources/compass.png", 300, 300)
        compass_panel.AddNeedle("resources/compassneedle.png", "heading")
        compass_panel.AddNeedle("resources/compassbearing.png", "bearing")
        sizer.Add(compass_panel, pos=(0, 4), span=(2, 2), flag=wx.EXPAND|wx.LEFT|wx.RIGHT, border=5)

        panel.SetSizer(sizer)
        panel.Layout()

        #call self.Refresh() from your OnKeyArrow and OnMotion events. Update your scene data in those methods and set some
        #  flag e.g. self.repaint_needed = True. Then in OnIdle repaint the scene if self.repaint_needed is True.


    def OnClose(self, event):
        self.Destroy()

    def OnAbout(self, event):
        dlg = AboutBox()
        dlg.ShowModal()
        dlg.Destroy()


app = wx.App(redirect=True)  # Error messages go to popup window
top = Frame("<<project>>")
top.Show()
app.MainLoop()
