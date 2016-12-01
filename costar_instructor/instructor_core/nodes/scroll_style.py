scroll_style = '''QScrollBar:vertical {

  border-color: rgb(227, 227, 227);
  border-width: 1px;
  border-style: solid;

  background-color: rgb(240, 240, 240);
  width: 15px;
  margin: 21px 0 21px 0;
}

QScrollBar::handle:vertical {

  background-color: rgb(200, 200, 200);
  min-height: 25px;

}

 QScrollBar::add-line:vertical {
    border: 1px solid grey;
  background-color: rgb(241, 241, 241);
    height: 20px;
    subcontrol-position: bottom;
    subcontrol-origin: margin;
}

QScrollBar::sub-line:vertical {
    border: 1px solid grey;
    background-color: rgb(241, 241, 241);
    height: 20px;
    subcontrol-position: top;
    subcontrol-origin: margin;
}'''