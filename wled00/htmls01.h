/*
 * Settings html
 */
const char PAGE_settings[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
<meta http-equiv=Content-Type content="text/html; charset=windows-1252">
<title>Settings</title>
<script>function GetCurrent(){var a=new XMLHttpRequest();a.onreadystatechange=function(){if(this.readyState==4){if(this.status==200){if(this.responseXML!=null){document.S_form.CSSID.value=this.responseXML.getElementsByTagName("cssid")[0].innerHTML;document.S_form.CPASS.value=this.responseXML.getElementsByTagName("cpass")[0].innerHTML;document.S_form.CSIP0.value=this.responseXML.getElementsByTagName("csips")[0].innerHTML;document.S_form.CSIP1.value=this.responseXML.getElementsByTagName("csips")[1].innerHTML;document.S_form.CSIP2.value=this.responseXML.getElementsByTagName("csips")[2].innerHTML;document.S_form.CSIP3.value=this.responseXML.getElementsByTagName("csips")[3].innerHTML;document.S_form.CSGW0.value=this.responseXML.getElementsByTagName("csgws")[0].innerHTML;document.S_form.CSGW1.value=this.responseXML.getElementsByTagName("csgws")[1].innerHTML;document.S_form.CSGW2.value=this.responseXML.getElementsByTagName("csgws")[2].innerHTML;document.S_form.CSGW3.value=this.responseXML.getElementsByTagName("csgws")[3].innerHTML;document.S_form.CSSN0.value=this.responseXML.getElementsByTagName("cssns")[0].innerHTML;document.S_form.CSSN1.value=this.responseXML.getElementsByTagName("cssns")[1].innerHTML;document.S_form.CSSN2.value=this.responseXML.getElementsByTagName("cssns")[2].innerHTML;document.S_form.CSSN3.value=this.responseXML.getElementsByTagName("cssns")[3].innerHTML;document.S_form.CMDNS.value=this.responseXML.getElementsByTagName("cmdns")[0].innerHTML;document.S_form.APSSID.value=this.responseXML.getElementsByTagName("apssid")[0].innerHTML;document.S_form.APHSSID.checked=(this.responseXML.getElementsByTagName("aphssid")[0].innerHTML)!=0?true:false;document.S_form.APPASS.value=this.responseXML.getElementsByTagName("appass")[0].innerHTML;document.S_form.APCHAN.value=this.responseXML.getElementsByTagName("apchan")[0].innerHTML;document.S_form.DESC.value=this.responseXML.getElementsByTagName("desc")[0].innerHTML;document.S_form.COLMD.checked=(this.responseXML.getElementsByTagName("colmd")[0].innerHTML)!=0?true:false;document.S_form.LEDCN.value=this.responseXML.getElementsByTagName("ledcn")[0].innerHTML;document.S_form.CLDFR.value=this.responseXML.getElementsByTagName("cldef")[0].innerHTML;document.S_form.CLDFG.value=this.responseXML.getElementsByTagName("cldef")[1].innerHTML;document.S_form.CLDFB.value=this.responseXML.getElementsByTagName("cldef")[2].innerHTML;document.S_form.CLDFA.value=this.responseXML.getElementsByTagName("cldfa")[0].innerHTML;document.S_form.CLDFW.value=this.responseXML.getElementsByTagName("cldfw")[0].innerHTML;document.S_form.FXDEF.value=this.responseXML.getElementsByTagName("fxdef")[0].innerHTML;document.S_form.SXDEF.value=this.responseXML.getElementsByTagName("sxdef")[0].innerHTML;document.S_form.BOOTN.checked=(this.responseXML.getElementsByTagName("bootn")[0].innerHTML)!=0?true:false;document.S_form.GCBRI.checked=(this.responseXML.getElementsByTagName("gcbri")[0].innerHTML)!=0?true:false;document.S_form.GCRGB.checked=(this.responseXML.getElementsByTagName("gcrgb")[0].innerHTML)!=0?true:false;document.S_form.BTNON.checked=(this.responseXML.getElementsByTagName("btnon")[0].innerHTML)!=0?true:false;document.S_form.TFADE.checked=(this.responseXML.getElementsByTagName("tfade")[0].innerHTML)!=0?true:false;document.S_form.TDLAY.value=this.responseXML.getElementsByTagName("tdlay")[0].innerHTML;document.S_form.TLBRI.value=this.responseXML.getElementsByTagName("tlbri")[0].innerHTML;document.S_form.TLDUR.value=this.responseXML.getElementsByTagName("tldur")[0].innerHTML;document.S_form.TLFDE.checked=(this.responseXML.getElementsByTagName("tlfde")[0].innerHTML)!=0?true:false;document.S_form.NUDPP.value=this.responseXML.getElementsByTagName("nudpp")[0].innerHTML;document.S_form.NRCVE.checked=(this.responseXML.getElementsByTagName("nrcve")[0].innerHTML)!=0?true:false;document.S_form.NRBRI.value=this.responseXML.getElementsByTagName("nrbri")[0].innerHTML;document.S_form.NSDIR.checked=(this.responseXML.getElementsByTagName("nsdir")[0].innerHTML)!=0?true:false;document.S_form.NSBTN.checked=(this.responseXML.getElementsByTagName("nsbtn")[0].innerHTML)!=0?true:false;document.S_form.ALEXA.checked=(this.responseXML.getElementsByTagName("alexa")[0].innerHTML)!=0?true:false;document.S_form.AINVN.value=this.responseXML.getElementsByTagName("ainvn")[0].innerHTML;document.S_form.NSALX.checked=(this.responseXML.getElementsByTagName("nsalx")[0].innerHTML)!=0?true:false;document.S_form.NTPON.checked=(this.responseXML.getElementsByTagName("ntpon")[0].innerHTML)!=0?true:false;document.getElementsByClassName("times")[0].innerHTML=this.responseXML.getElementsByTagName("times")[0].innerHTML;document.S_form.OLDEF.value=this.responseXML.getElementsByTagName("oldef")[0].innerHTML;document.S_form.WOFFS.value=this.responseXML.getElementsByTagName("woffs")[0].innerHTML;document.S_form.WOFFN.checked=(this.responseXML.getElementsByTagName("woffn")[0].innerHTML)!=0?true:false;document.S_form.NOOTA.checked=(this.responseXML.getElementsByTagName("noota")[0].innerHTML)!=0?true:false;document.S_form.NORAP.checked=(this.responseXML.getElementsByTagName("norap")[0].innerHTML)!=0?true:false;document.getElementsByClassName("sip")[0].innerHTML=this.responseXML.getElementsByTagName("sip")[0].innerHTML;document.getElementsByClassName("sip")[1].innerHTML=this.responseXML.getElementsByTagName("sip")[1].innerHTML;document.getElementsByClassName("msg")[0].innerHTML=this.responseXML.getElementsByTagName("msg")[0].innerHTML}}}};a.open("GET","/get-settings",true);a.send(null)};</script>
<style>body{line-height:150%}</style>
</head>
<body onload=GetCurrent() class=__plain_text_READY__>
<h1 style=text-align:center>WLED Settings</h1>
<form id=form_s name=S_form action=set-settings method=post>
<div align=center><input type=submit name=SUBM value=Save></div>
<hr>
<h2>WiFi setup</h2>
<h3>Connect to existing network</h3>
Network SSID (leave empty to not connect): <br><input name=CSSID maxlength=32> <br>
Network password: <br> <input type=password name=CPASS maxlength=63> <br>
Static IP (leave at 0.0.0.0 for DHCP): <br>
<input name=CSIP0 maxlength=3 size=2> .
<input name=CSIP1 maxlength=3 size=2> .
<input name=CSIP2 maxlength=3 size=2> .
<input name=CSIP3 maxlength=3 size=2> <br>
Static gateway: <br>
<input name=CSGW0 maxlength=3 size=2> .
<input name=CSGW1 maxlength=3 size=2> .
<input name=CSGW2 maxlength=3 size=2> .
<input name=CSGW3 maxlength=3 size=2> <br>
Static subnet mask: <br>
<input name=CSSN0 maxlength=3 size=2> .
<input name=CSSN1 maxlength=3 size=2> .
<input name=CSSN2 maxlength=3 size=2> .
<input name=CSSN3 maxlength=3 size=2> <br>
mDNS address (leave empty for no mDNS): <br/>
http:// <input name=CMDNS maxlength=32> .local <br>
Client IP: <span class=sip> Not connected </span> <br>
<h3>Configure Access Point</h3>
AP SSID (leave empty for no AP): <br> <input name=APSSID maxlength=32> <br>
Hide AP SSID: <input type=checkbox name=APHSSID> <br>
AP password (leave empty for open): <br> <input type=password name=APPASS maxlength=63> <br>
AP channel: <input name=APCHAN maxlength=2 size=2> <br>
AP IP: <span class=sip> Not active </span> <br>
<hr>
<h2>Application setup</h2>
<h3>Web setup</h3>
Server description: <input name=DESC maxlength=32> <br>
Use HSB sliders instead of RGB by default: <input type=checkbox name=COLMD> <br>
<h3>LED setup</h3>
LED count (max. 255): <input name=LEDCN maxlength=3 size=2> <br>
Default RGB color:
<input name=CLDFR maxlength=3 size=2>
<input name=CLDFG maxlength=3 size=2>
<input name=CLDFB maxlength=3 size=2> <br>
Default brightness: <input name=CLDFA maxlength=3 size=2> (0-255) <br>
Default white value (only RGBW, -1 to disable): <input name=CLDFW maxlength=3 size=2> <br>
Default effect ID: <input name=FXDEF maxlength=3 size=2> <br>
Default effect speed: <input name=SXDEF maxlength=3 size=2> <br>
Ignore and use current color, brightness and effects: <input type=checkbox name=CBEOR> <br>
Turn on after power up/reset: <input type=checkbox name=BOOTN> <br>
Use Gamma correction for brightness: <input type=checkbox name=GCBRI> <br>
Use Gamma correction for color: <input type=checkbox name=GCRGB> <br>
Brightness factor: <input name=NRBRI maxlength=3 size=2> % <br>
<h3>Button setup</h3>
On/Off button enabled: <input type=checkbox name=BTNON> <br>
<h3>Transitions</h3>
Fade: <input type=checkbox name=TFADE> <br>
Transition Delay: <input name=TDLAY maxlength=5 size=2> ms <br>
<h3>Timed light</h3>
Target brightness: <input name=TLBRI maxlength=3 size=2> (0-255) <br>
Change after: <input name=TLDUR maxlength=3 size=2> min <br>
Fade: <input type=checkbox name=TLFDE> <br>
<h3>Broadcast</h3>
UDP Port: <input name=NUDPP maxlength=5 size=2><br>
Receive notifications: <input type=checkbox name=NRCVE> <br>
Send notifications on direct change: <input type=checkbox name=NSDIR> <br>
Send notifications on button press: <input type=checkbox name=NSBTN> <br>
<h3>Interfaces</h3>
Emulate Alexa device: <input type=checkbox name=ALEXA> <br>
Alexa invocation name: <input name=AINVN maxlength=32><br>
Send Alexa notifications: <input type=checkbox name=NSALX> <br>
<h3>Time (experimental!)</h3>
NTP was updated but still causes crashes. Requires reboot. <br>
Get time from NTP server: <input type=checkbox name=NTPON> <br>
Current local time is <span class=times>unknown</span> <br>
<h3>Advanced</h3>
Default overlay ID: <input name=OLDEF maxlength=3 size=2> <br>
WARLS offset: <input name=WOFFS maxlength=3 size=2> negative <input type=checkbox name=WOFFN><br>
<h3>Security</h3>
OTA locked: <input type=checkbox name=NOOTA> <br>
Passphrase: <input type=password name=OPASS maxlength=32> <br>
To enable OTA, for security reasons you need to also enter the correct password! <br>
The password may/should be changed when OTA is enabled. <br>
Disable OTA when not in use, otherwise an attacker could reflash device software! <br> <br>
Disable recovery AP (<i>Not implemented</i>): <input type=checkbox name=NORAP> <br>
In case of a connection error there will be no wireless recovery possible! <br>
Completely disables all Access Point functions. <br> <br>
Factory reset: <input type=checkbox name=RESET> <br>
All EEPROM content (settings) will be erased. <br> <br>
HTTP traffic is not encrypted. An attacker in the same network could intercept form data!<br>
<h3>About</h3>
WLED version 0.4p <br>
(c) 2016-2017 Christian Schwinne <br>
<i>Licensed under the MIT license</i> <br>
<i>Uses libraries:</i> <br>
<i>ESP8266 Arduino Core</i> <br>
<i>WS2812FX by kitesurfer1404 (Aircoookie fork)</i> <br>
<i>Timezone library by JChristensen</i> <br>
<i>arduino-esp8266-alexa-multiple-wemo-switch by kakopappa</i> <br>
Server message: <span class=msg> XML response error! </span>
<br><br><hr>
<div align=center><input type=submit name=SUBM value=Save></div>
</form>
</body>
</html>
)=====";
/*
 * Settings set html
 */
const char PAGE_settingssaved[] PROGMEM = R"=====(
<!DOCTYPE html>
<html><head>
  <meta http-equiv="Content-Type" content="text/html; charset=windows-1252">
  <title>Saved Settings</title>
  <script>
    function OpenReboot()
    {
      window.open("/reset","_self");
    }
  </script>
  </head><body>
  <div align="center">
    <h2>Settings saved.</h2>
    <p>If you made changes to WiFi configuration, please reboot.</p><br>
    <input type="button" value="Reboot" onclick="OpenReboot()">
  </div></body>
</html>
)=====";
