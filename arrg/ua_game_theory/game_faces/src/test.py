import time
import wx

class CustomCheckBox(wx.PyControl):
    """
    A custom class that replicates some of the functionalities of wx.CheckBox,
    while being completely owner-drawn with a nice check bitmaps.
    """

    def __init__(self, parent, id=wx.ID_ANY, label="", pos=wx.DefaultPosition,
                 size=wx.DefaultSize, style=wx.NO_BORDER, validator=wx.DefaultValidator,
                 name="CustomCheckBox"):
        """
        Default class constructor.

        @param parent: Parent window. Must not be None.
        @param id: CustomCheckBox identifier. A value of -1 indicates a default value.
        @param label: Text to be displayed next to the checkbox.
        @param pos: CustomCheckBox position. If the position (-1, -1) is specified
                    then a default position is chosen.
        @param size: CustomCheckBox size. If the default size (-1, -1) is specified
                     then a default size is chosen.
        @param style: not used in this demo, CustomCheckBox has only 2 state
        @param validator: Window validator.
        @param name: Window name.
        """

        # Ok, let's see why we have used wx.PyControl instead of wx.Control.
        # Basically, wx.PyControl is just like its wxWidgets counterparts
        # except that it allows some of the more common C++ virtual method
        # to be overridden in Python derived class. For CustomCheckBox, we
        # basically need to override DoGetBestSize and AcceptsFocusFromKeyboard

        wx.PyControl.__init__(self, parent, id, pos, size, style, validator, name)

        # Initialize our cool bitmaps
        #self.InitializeBitmaps()

        # Initialize the focus pen colour/dashes, for faster drawing later
        #self.InitializeColours()

        # By default, we start unchecked
        self._checked = False

        # Set the spacing between the check bitmap and the label to 3 by default.
        # This can be changed using SetSpacing later.
        self._spacing = 3

        # I assume at the beginning we are not focused
        self._hasFocus = False

        # Ok, set the wx.PyControl label, its initial size (formerly known an
        # SetBestFittingSize), and inherit the attributes from the standard
        # wx.CheckBox
        self.SetLabel(label)
        self.SetInitialSize(size)
        self.InheritAttributes()

        # Bind the events related to our control: first of all, we use a
        # combination of wx.BufferedPaintDC and an empty handler for
        # wx.EVT_ERASE_BACKGROUND (see later) to reduce flicker
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_ERASE_BACKGROUND, self.OnEraseBackground)

        # Then we want to monitor user clicks, so that we can switch our
        # state between checked and unchecked
        self.Bind(wx.EVT_LEFT_DOWN, self.OnMouseClick)
        if wx.Platform == '__WXMSW__':
            # MSW Sometimes does strange things...
            self.Bind(wx.EVT_LEFT_DCLICK,  self.OnMouseClick)
    	self.Bind(wx.EVT_ENTER_WINDOW, self.OnMouseOver)
        self.Bind(wx.EVT_LEAVE_WINDOW, self.OnMouseLeave)

        # We want also to react to keyboard keys, namely the
        # space bar that can toggle our checked state
        self.Bind(wx.EVT_KEY_UP, self.OnKeyUp)

        # Then, we react to focus event, because we want to draw a small
        # dotted rectangle around the text if we have focus
        # This might be improved!!!
        self.Bind(wx.EVT_SET_FOCUS, self.OnSetFocus)
        self.Bind(wx.EVT_KILL_FOCUS, self.OnKillFocus)

    def OnPaint(self, event):
        """ Handles the wx.EVT_PAINT event for CustomCheckBox. """

        # If you want to reduce flicker, a good starting point is to
        # use wx.BufferedPaintDC.
        dc = wx.BufferedPaintDC(self)

        # It is advisable that you don't overcrowd the OnPaint event
        # (or any other event) with a lot of code, so let's do the
        # actual drawing in the Draw() method, passing the newly
        # initialized wx.BufferedPaintDC
        self.Draw(dc)


    def Draw(self, dc):
        """
        Actually performs the drawing operations, for the bitmap and
        for the text, positioning them centered vertically.
        """

        # Get the actual client size of ourselves
        width, height = self.GetClientSize()

        if not width or not height:
            # Nothing to do, we still don't have dimensions!
            return

        # Initialize the wx.BufferedPaintDC, assigning a background
        # colour and a foreground colour (to draw the text)
        backColour = self.GetBackgroundColour()
        backBrush = wx.Brush(backColour, wx.SOLID)
        dc.SetBackground(backBrush)
        dc.Clear()

        if self.IsEnabled():
            dc.SetTextForeground(self.GetForegroundColour())
        else:
            dc.SetTextForeground(wx.SystemSettings.GetColour(wx.SYS_COLOUR_GRAYTEXT))

        dc.SetFont(self.GetFont())

        # Get the text label for the checkbox, the associated check bitmap
        # and the spacing between the check bitmap and the text
        label = self.GetLabel()
        bitmap = self.GetBitmap()
        spacing = self.GetSpacing()

        # Measure the text extent and get the check bitmap dimensions
        textWidth, textHeight = dc.GetTextExtent(label)
        bitmapWidth, bitmapHeight = bitmap.GetWidth(), bitmap.GetHeight()

        # Position the bitmap centered vertically
        bitmapXpos = 0
        bitmapYpos = (height - bitmapHeight)/2

        # Position the text centered vertically
        textXpos = bitmapWidth + spacing
        textYpos = (height - textHeight)/2

        # Draw the bitmap on the DC
        dc.DrawBitmap(bitmap, bitmapXpos, bitmapYpos, True)

        # Draw the text
        dc.DrawText(label, textXpos, textYpos)

        # Let's see if we have keyboard focus and, if this is the case,
        # we draw a dotted rectangle around the text (Windows behavior,
        # I don't know on other platforms...)
        if self.HasFocus():
            # Yes, we are focused! So, now, use a transparent brush with
            # a dotted black pen to draw a rectangle around the text
            dc.SetBrush(wx.TRANSPARENT_BRUSH)
            dc.SetPen(self._focusIndPen)
            dc.DrawRectangle(textXpos, textYpos, textWidth, textHeight)


    def OnEraseBackground(self, event):
        """ Handles the wx.EVT_ERASE_BACKGROUND event for CustomCheckBox. """

        # This is intentionally empty, because we are using the combination
        # of wx.BufferedPaintDC + an empty OnEraseBackground event to
        # reduce flicker
        pass

    def OnMouseClick(self, event):
        """ Handles the wx.EVT_LEFT_DOWN event for CustomCheckBox. """

        if not self.IsEnabled():
            # Nothing to do, we are disabled
            return

        self.SendCheckBoxEvent()
        event.Skip()

    def OnMouseOver(self, event):
        self.SetBackgroundColour((255, 0, 0))

    def OnMouseLeave(self, event):
        self.SetBackgroundColour((0, 0, 255))

    def SendCheckBoxEvent(self):
        """ Actually sends the wx.wxEVT_COMMAND_CHECKBOX_CLICKED event. """

        # This part of the code may be reduced to a 3-liner code
        # but it is kept for better understanding the event handling.
        # If you can, however, avoid code duplication; in this case,
        # I could have done:
        #
        # self._checked = not self.IsChecked()
        # checkEvent = wx.CommandEvent(wx.wxEVT_COMMAND_CHECKBOX_CLICKED,
        #                              self.GetId())
        # checkEvent.SetInt(int(self._checked))
        if self.IsChecked():

            # We were checked, so we should become unchecked
            self._checked = False

            # Fire a wx.CommandEvent: this generates a
            # wx.wxEVT_COMMAND_CHECKBOX_CLICKED event that can be caught by the
            # developer by doing something like:
            # MyCheckBox.Bind(wx.EVT_CHECKBOX, self.OnCheckBox)
            checkEvent = wx.CommandEvent(wx.wxEVT_COMMAND_CHECKBOX_CLICKED,
                                         self.GetId())

            # Set the integer event value to 0 (we are switching to unchecked state)
            checkEvent.SetInt(0)

        else:

            # We were unchecked, so we should become checked
            self._checked = True

            checkEvent = wx.CommandEvent(wx.wxEVT_COMMAND_CHECKBOX_CLICKED,
                                         self.GetId())

            # Set the integer event value to 1 (we are switching to checked state)
            checkEvent.SetInt(1)

        # Set the originating object for the event (ourselves)
        checkEvent.SetEventObject(self)

        # Watch for a possible listener of this event that will catch it and
        # eventually process it
        self.GetEventHandler().ProcessEvent(checkEvent)

        # Refresh ourselves: the bitmap has changed
        self.Refresh()

    def OnKeyUp(self, event):
        """ Handles the wx.EVT_KEY_UP event for CustomCheckBox. """

        if event.GetKeyCode() == wx.WXK_SPACE:
            # The spacebar has been pressed: toggle our state
            self.SendCheckBoxEvent()
            event.Skip()
            return

        event.Skip()

    def OnSetFocus(self, event):
        """ Handles the wx.EVT_SET_FOCUS event for CustomCheckBox. """

        self._hasFocus = True

        # We got focus, and we want a dotted rectangle to be painted
        # around the checkbox label, so we refresh ourselves
        self.Refresh()

    def OnKillFocus(self, event):
        """ Handles the wx.EVT_KILL_FOCUS event for CustomCheckBox. """

        self._hasFocus = False

        # We lost focus, and we want a dotted rectangle to be cleared
        # around the checkbox label, so we refresh ourselves
        self.Refresh()


class MyFrame(wx.Frame):

    def __init__(self, parent, id, title):
        wx.Frame.__init__(self, parent, id, title,
                         (100, 100), (400, 300))

        # self.Bind(wx.EVT_SIZE, self.OnSize)
        # self.Bind(wx.EVT_MOVE, self.OnMove)
        self.Bind(wx.EVT_CLOSE, self.OnCloseWindow)
        # self.Bind(wx.EVT_IDLE, self.OnIdle)

        self.count = 0

        panel = wx.Panel(self)
        sizer = wx.FlexGridSizer(cols=2, hgap=5, vgap=5)
        
        self.balanceCtrl = wx.TextCtrl(panel, -1, "10", style=wx.TE_READONLY)
        sizer.Add(wx.StaticText(panel, -1, "Balance:"))
        sizer.Add(self.balanceCtrl)

        #self.biddingCtrl = wx.TextCtrl(panel, -1, "")
        
        self.biddingCtrl = wx.ComboBox(panel, -1, "0",
                         choices=map(str, range(0, 10)),
                         style=wx.CB_DROPDOWN | wx.CB_READONLY
                         #| wx.TE_PROCESS_ENTER
                         #| wx.CB_SORT
                         )
        
        sizer.Add(wx.StaticText(panel, -1, "Your Bidding:"))
        sizer.Add(self.biddingCtrl)
        
        self.biddingButton = wx.Button(panel, -1, "Bid!")
        self.Bind(wx.EVT_BUTTON, self.OnClickBid, self.biddingButton)
        sizer.Add(wx.StaticText(panel, -1, ""))
        sizer.Add(self.biddingButton)

	self.customCheckBox = CustomCheckBox(panel, -1, "test")
	sizer.Add(self.customCheckBox)

        border = wx.BoxSizer()
        border.Add(sizer, 0, wx.ALL, 20)
        panel.SetSizer(border)
        

    def OnCloseWindow(self, event):
        app.keepGoing = False
        self.Destroy()

    # def OnIdle(self, event):
    #     self.idleCtrl.SetValue(str(self.count))
    #     self.count = self.count + 1
    # 
    # def OnSize(self, event):
    #     size = event.GetSize()
    #     self.balanceCtrl.SetValue("%s, %s" % (size.width, size.height))
    #     event.Skip()
    # 
    # def OnMove(self, event):
    #     pos = event.GetPosition()
    #     self.biddingCtrl.SetValue("%s, %s" % (pos.x, pos.y))

    def OnClickBid(self, event):
        print event


class MyApp(wx.App):
    def MainLoop(self):

        # Create an event loop and make it active.  If you are
        # only going to temporarily have a nested event loop then
        # you should get a reference to the old one and set it as
        # the active event loop when you are done with this one...
        evtloop = wx.EventLoop()
        old = wx.EventLoop.GetActive()
        wx.EventLoop.SetActive(evtloop)

        # This outer loop determines when to exit the application,
        # for this example we let the main frame reset this flag
        # when it closes.
        while self.keepGoing:
            # At this point in the outer loop you could do
            # whatever you implemented your own MainLoop for.  It
            # should be quick and non-blocking, otherwise your GUI
            # will freeze.  

            # call_your_code_here()


            # This inner loop will process any GUI events
            # until there are no more waiting.
            while evtloop.Pending():
                evtloop.Dispatch()

            # Send idle events to idle handlers.  You may want to
            # throttle this back a bit somehow so there is not too
            # much CPU time spent in the idle handlers.  For this
            # example, I'll just snooze a little...
            time.sleep(0.10)
            self.ProcessIdle()

        wx.EventLoop.SetActive(old)



    def OnInit(self):
        frame = MyFrame(None, -1, "This is a test")
        frame.Show(True)
        self.SetTopWindow(frame)

        self.keepGoing = True
        return True


app = MyApp(False)
app.MainLoop()

