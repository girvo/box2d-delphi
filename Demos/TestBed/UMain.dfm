object frmMain: TfrmMain
  Left = 282
  Top = 108
  Caption = 'TestBed'
  ClientHeight = 586
  ClientWidth = 747
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'Tahoma'
  Font.Style = []
  KeyPreview = True
  OldCreateOrder = False
  WindowState = wsMaximized
  OnCreate = FormCreate
  OnDestroy = FormDestroy
  OnKeyDown = FormKeyDown
  OnKeyUp = FormKeyUp
  OnMouseWheelDown = FormMouseWheelDown
  OnMouseWheelUp = FormMouseWheelUp
  PixelsPerInch = 96
  TextHeight = 13
  object Panel1: TPanel
    Left = 584
    Top = 0
    Width = 163
    Height = 586
    Align = alRight
    ParentBackground = False
    TabOrder = 0
    object Label1: TLabel
      Left = 8
      Top = 5
      Width = 26
      Height = 13
      Caption = 'Tests'
    end
    object Label2: TLabel
      Left = 8
      Top = 100
      Width = 37
      Height = 13
      Caption = 'Visibility'
    end
    object cboTests: TComboBox
      Left = 7
      Top = 24
      Width = 148
      Height = 21
      AutoDropDown = True
      AutoCloseUp = True
      Style = csDropDownList
      DropDownCount = 100
      ImeName = 'Chinese (Simplified) - US Keyboard'
      ItemHeight = 13
      TabOrder = 0
      OnChange = cboTestsChange
      OnCloseUp = cboTestsCloseUp
    end
    object chkWarmStarting: TCheckBox
      Left = 8
      Top = 51
      Width = 97
      Height = 17
      Caption = 'Warm Starting'
      Checked = True
      State = cbChecked
      TabOrder = 1
      OnClick = SimulationOptionsChanged
    end
    object chkTimeOfImpact: TCheckBox
      Left = 8
      Top = 66
      Width = 105
      Height = 17
      Caption = 'Time of Impact'
      Checked = True
      State = cbChecked
      TabOrder = 2
      OnClick = SimulationOptionsChanged
    end
    object chklstVisibility: TCheckListBox
      Left = 8
      Top = 118
      Width = 148
      Height = 158
      OnClickCheck = chklstVisibilityClickCheck
      ImeName = 'Chinese (Simplified) - US Keyboard'
      ItemHeight = 13
      Items.Strings = (
        'Shapes'
        'Joints'
        'AABBs'
        'Pairs'
        'Contact Points'
        'Contact Normals'
        'Contact Forces'
        'Friction Forces'
        'Center of Masses'
        'Statistics'
        'Key Information')
      TabOrder = 3
    end
    object btnPause: TButton
      Left = 7
      Top = 418
      Width = 75
      Height = 25
      Caption = 'Pause'
      TabOrder = 4
      OnClick = btnPauseClick
    end
    object btnSingleStep: TButton
      Left = 82
      Top = 418
      Width = 75
      Height = 25
      Caption = 'Single Step'
      TabOrder = 5
      OnClick = btnSingleStepClick
    end
    object GroupBox1: TGroupBox
      Left = 8
      Top = 283
      Width = 148
      Height = 70
      Caption = 'Gravity'
      TabOrder = 6
      object Label3: TLabel
        Left = 11
        Top = 21
        Width = 6
        Height = 13
        Caption = 'X'
      end
      object Label4: TLabel
        Left = 11
        Top = 45
        Width = 6
        Height = 13
        Caption = 'Y'
      end
      object editGravityX: TEdit
        Left = 20
        Top = 18
        Width = 51
        Height = 21
        ImeName = 'Chinese (Simplified) - US Keyboard'
        TabOrder = 0
      end
      object editGravityY: TEdit
        Left = 20
        Top = 42
        Width = 51
        Height = 21
        ImeName = 'Chinese (Simplified) - US Keyboard'
        TabOrder = 1
      end
      object btnConfirmGravity: TButton
        Left = 80
        Top = 16
        Width = 63
        Height = 25
        Caption = 'Confirm'
        TabOrder = 2
        OnClick = btnConfirmGravityClick
      end
    end
    object btnReset: TButton
      Left = 7
      Top = 443
      Width = 75
      Height = 25
      Caption = 'Reset'
      TabOrder = 7
      OnClick = btnResetClick
    end
    object GroupBox2: TGroupBox
      Left = 8
      Top = 355
      Width = 148
      Height = 57
      Caption = 'Mode'
      TabOrder = 8
      object rdoRealTime: TRadioButton
        Left = 8
        Top = 16
        Width = 113
        Height = 17
        Caption = 'Real Time'
        Checked = True
        TabOrder = 0
        TabStop = True
        OnClick = rdoRealTimeClick
      end
      object rdoFixedStep: TRadioButton
        Left = 8
        Top = 34
        Width = 113
        Height = 17
        Caption = 'Fixed Step(1/60s)'
        TabOrder = 1
        OnClick = rdoFixedStepClick
      end
    end
    object chkAntialiasing: TCheckBox
      Left = 8
      Top = 474
      Width = 97
      Height = 17
      Caption = 'Antialiasing'
      Checked = True
      State = cbChecked
      TabOrder = 9
      OnClick = chkAntialiasingClick
    end
    object chkSubStepping: TCheckBox
      Left = 8
      Top = 81
      Width = 105
      Height = 17
      Caption = 'Sub-Stepping'
      TabOrder = 10
      OnClick = SimulationOptionsChanged
    end
  end
end
