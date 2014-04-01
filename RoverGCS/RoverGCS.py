#!/usr/bin/python

# DIY Rovers Ground Control Station
#


# -*- coding: <<encoding>> -*-
#-------------------------------------------------------------------------------
#   <<project>>
#
#-------------------------------------------------------------------------------

import wxversion

wxversion.select("3.0")
import wx, wx.html
import sys

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


class ImagePanel(wx.Panel):
    def __init__(self, parent, imagepath, pos=(0, 0), size=(100,100)):
        wx.Panel.__init__(self, parent)
        image = wx.Image(imagepath)
        image = image.Scale(size[0], size[1])
        wx.StaticBitmap(self, -1, wx.BitmapFromImage(image), (0,0))


class Frame(wx.Frame):
    def __init__(self, title):
        wx.Frame.__init__(self, None, title=title, pos=(150, 150), size=(955, 400))
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        menuBar = wx.MenuBar()
        menu = wx.Menu()
        m_exit = menu.Append(wx.ID_EXIT, "E&xit\tAlt-X", "Close window and exit program.")
        self.Bind(wx.EVT_MENU, self.OnClose, m_exit)
        menuBar.Append(menu, "&File")
        menu = wx.Menu()
        m_about = menu.Append(wx.ID_ABOUT, "&About", "Information about this program")
        self.Bind(wx.EVT_MENU, self.OnAbout, m_about)
        menuBar.Append(menu, "&Help")
        self.SetMenuBar(menuBar)

        self.statusbar = self.CreateStatusBar()

        panel = wx.Panel(self)
        box = wx.BoxSizer(wx.VERTICAL)

        m_text = wx.StaticText(panel, -1, "Hello World!")
        m_text.SetFont(wx.Font(14, wx.SWISS, wx.NORMAL, wx.BOLD))
        m_text.SetSize(m_text.GetBestSize())
        box.Add(m_text, 0, wx.ALL, 10)

        m_close = wx.Button(panel, wx.ID_CLOSE, "Close")
        m_close.Bind(wx.EVT_BUTTON, self.OnClose)
        box.Add(m_close, 0, wx.ALL, 10)

        bmp = wx.Image("resources/maple.jpg", wx.BITMAP_TYPE_ANY).ConvertToBitmap()
        self.bitmap1 = wx.StaticBitmap(self, -1, bmp, (0, 0))

        self.add_image("resources/speedometer1.png", 300, 300, (5, 5))
        self.add_image("resources/speedometerneedle1.png", 300, 300, (5, 5))
        #        img = bitmap.ConvertToImage()
        #    img_centre = wx.Point( img.GetWidth()/2, img.GetHeight()/2 )
        #    img = img.Rotate( angle, img_centre )
        #    dc.WriteBitmap( img.ConvertToBitmap(), 0, 0 )

        self.add_image("resources/voltmeter1.png", 150, 150, (320, 5))
        self.add_image("resources/voltmeterneedle1.png", 150, 150, (320, 5))
        self.add_image("resources/ammeter2.gif", 150, 150, (480, 5))
        self.add_image("resources/ammeterneedle2.gif", 150, 150, (480, 5))
        self.add_image("resources/fuel1.png", 150, 150, (320, 160))
        self.add_image("resources/fuelneedle1.png", 150, 150, (320, 160))
        self.add_image("resources/clock.png", 150, 150, (480, 160))
        self.add_image("resources/clockhour.png", 150, 150, (480, 160))
        self.add_image("resources/clockminute.png", 150, 150, (480, 160))
        self.add_image("resources/clocksecond.png", 150, 150, (480, 160))
        self.add_image("resources/compass.png", 300, 300, (650, 5))
        #self.add_image("resources/compassneedle.png", 300, 300, (650, 5))
        ipanel = ImagePanel(self, "resources/compassneedle.png", (650, 5), (300,300))
        self.add_image("resources/compassbearing.png", 300, 300, (650, 5))

        #ll self.Refresh() from your OnKeyArrow and OnMotion events. Update your scene data in those methods and set some
        #  flag e.g. self.repaint_needed = True. Then in OnIdle repaint the scene if self.repaint_needed is True.

        panel.SetSizer(box)
        panel.Layout()

    def add_image(self, path, width, height, pos):
        image = wx.Image(path)
        image = image.Scale(width, height)
        wx.StaticBitmap(self, -1, wx.BitmapFromImage(image), pos)

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