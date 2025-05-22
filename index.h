const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
   <meta charset="utf-8">
   <meta name="viewport" 
      content="width=device-width, initial-scale=0.4, maximum-scale=0.4, user-scalable=0"/>
<title>Climate Control</title>
</head>

<style>
   :root {
  --shadow-color:var(--blue-70);

  --verdana-font: 'Brush Script MT',sans-serif, Helvetica; 
  --serif-font: serif;
  --system-font: system-ui;

  --red: #FC4444;
  --green: #1DD63B;
  
  --black: #0B0C10;
  --grey30: #202833;
  --grey60: #46A29F;
  --grey80:#C5C6C8;
  --blue: #66c5fc; 
  --white: #ffffff;

  /*--color1: #3FEEE7;
  --color2: #55BDCA;
  --color3: #96CAEF;
  --color4: #CAFAFE;*/
  background: var(--grey30);
}
*{
  padding: 0;
  margin: 0;
  font-family: var(--system-font) ;
  font-size: clamp(1.7rem, 2vw,3rem);
  -webkit-tap-highlight-color: transparent;
}

.prevent-select {
  -webkit-user-select: none; /* Safari */
  -ms-user-select: none; /* IE 10 and IE 11 */
  user-select: none; /* Standard syntax */
}

html,body{
    height: 100%;  
}

#app-main{
  min-height: 0; /* stretch */
  box-sizing:border-box;
  align-items: stretch;
  margin: 5%;
  background: var(--grey30);
}

.app-header{
   padding: 3%;
   align-self: center;
   font-size: clamp(2.5rem, 2vw,3rem);
   font-family: serif;
   text-shadow: 0px 0px 10px var(--blue);
}

.indicators-list{
  
   width: 100%;
  box-sizing: border-box;
  display: flex;
  flex-wrap: wrap;  /* horisontal list continues downwards */
  align-items: start; 
  flex-shrink:1;
}

.indicators-list_item{

  width: 48%;
  box-sizing: border-box;
  vertical-align: middle;
  padding: 0.5% 2vw;
  margin: 0.5% 1%;
  border: 0px solid var(--black);
  border-radius: 1vw;
}

.indicator_title{
   width: 50%;
   height: 100%;
}
.indicator_value{
   vertical-align: baseline;
   width: 50%;
   height: 100%
}

.inputs-list{
width: 100%;
  box-sizing: border-box;
  display: flex;
  flex-wrap: wrap;  /* horisontal list continues downwards */
  align-items: start; 
  flex-shrink:1;
}

.inputs-list_item{
   width: 100%;
   box-sizing: border-box;
   align-items: center;
   padding: 0.5% 2vw;
   margin: 0.5% 1%;
   display: flex;
   
   border: 0px solid var(--black);
   border-radius: 1vw;
}

.input_title{
   width: 70%;
}

.input_field{
   width: 15%;
   margin-left: auto; 
   margin-right: 0;
   
}

input[type=text] {
  box-sizing: border-box;
  border: 1px solid var(--black);
  border-radius: 1vw;
  background-color: var(--white);
  color: var(--grey30);
  
  text-align: center;
}

.button-row{
  display: flex;
  width: 100%;
  
  column-gap: 2%;
  align-items: center;
  margin: 2% 0%;
  

}

.button-row_item{

}
.button{
   
   cursor: pointer;
   border-radius: 1dvb;
   
   width: 100%;
   height: 12vw;;
   text-align: center;
   align-items: center;
   display: grid;
   
}



.save-button{
    background-color: var(--red);
    box-shadow: 0px 0px 5px var(--red);
    border-color: var(--red);
    transition: box-shadow 0.5s ease-in-out;
}
.save-button.clicked{
    box-shadow: 0px 0px 50px var(--red);
    transition: box-shadow 0s ease-in-out;
}

.load-button{
    background-color: var(--red);
    box-shadow: 0px 0px 5px var(--red);
    border-color: var(--red);
    transition: box-shadow 0.5s ease-in-out;
}
.load-button.clicked{
    box-shadow: 0px 0px 50px var(--red);
    transition: box-shadow 0s ease-in-out;
}

.set-button{
    background-color: var(--blue);
    box-shadow: 0px 0px 5px var(--blue);
    border-color: var(--blue);
    transition: box-shadow 0.5s ease-in-out;
}
.set-button.clicked{
    box-shadow: 0px 0px 50px var(--blue);
    transition: box-shadow 0s ease-in-out;
}

.get-button{
    background-color: var(--blue);
    box-shadow: 0px 0px 5px var(--blue);
    border-color: var(--blue);
    transition: box-shadow 0.5s ease-in-out;
}
.get-button.clicked{
   box-shadow: 0px 0px 50px var(--blue);
   transition: box-shadow 0s ease-in-out;

}

separator{
   padding: 1%;
}
vseparator{
   padding: 0% 1%;
}
.text{
   color: var(--white);
    
    /*text-shadow: 0px 10px 10px var(--black);*/
    -webkit-user-select: none; /* Safari */
    -ms-user-select: none; /* IE 10 and IE 11 */
    user-select: none; /* Standard syntax */
}

.k_text{
   color: var(--red  );
   
    /*text-shadow: 0px 10px 10px var(--black);*/
    -webkit-user-select: none; /* Safari */
    -ms-user-select: none; /* IE 10 and IE 11 */
    user-select: none; /* Standard syntax */
    text-shadow: 0px 0px 80px var(--red);
}

.number{
   color: var(--blue);
   font-size: clamp(1.9rem, 2vw,3rem);
   text-align: center;
    /*text-shadow: 0px 10px 10px var(--black);*/
    -webkit-user-select: none; /* Safari */
    -ms-user-select: none; /* IE 10 and IE 11 */
    user-select: none; /* Standard syntax */
    text-shadow: 0px 0px 120px var(--blue);
}
</style>

<body>
   <main class="app-main">
      <div class="app-header text ">
         HONDA CLIMATE CONTROL
      </div>

      <div class="list indicators-list">
         
         <div class="indicators-list_item indicator">
            <text class="indicator_title text"n>Target salon t:</text>
            <text id="wanted_temp" class="indicator_value number">122</text>
         </div>

         <div class="indicators-list_item indicator">
            <text  class="indicator_title text">Pot percent:</text>
            <text  id="pot_percent" class="indicator_value number">123</text>
         </div>



         <div class="indicators-list_item indicator">
            <text class="indicator_title text">Salon t:</text>
            <text id="temp3" class="indicator_value number">124</text>
         </div>

         <div class="indicators-list_item indicator">
            <text class="indicator_title text">Street t:</text>
            <text id="temp2" class="indicator_value number">1</text>
         </div>


         <div class="indicators-list_item indicator">
            <text class="indicator_title text">Target stream t:</text>
            <text id="wanted_temp2" class="indicator_value number">1</text>
         </div>

         <div class="indicators-list_item indicator">
            <text class="indicator_title text">Light level:</text>
            <text id="light_level" class="indicator_value number">1</text>
         </div>


         <div class="indicators-list_item indicator">
            <text class="indicator_title text">Stream t:</text>
            <text id="temp1" class="indicator_value number">0</text>
         </div>

         <div class="indicators-list_item indicator">
            <text class="indicator_title text">Servo Percent:</text>
            <text id="servo_percent" class="indicator_value number">1</text>
         </div>

      </div>

      <separator></separator>

      <form class="list inputs-list" name="k_form">
         <div class="inputs-list_item">
            <div class="input_title">
              <text  fill="#777" class="k_text" >k1</text>
              <text  fill="#777" class="text" >Допуск, 0 для мануального режима</text>
            </div>

            <input type="text" name="k1" id="k1" value="1" class="input_field"><br>
         </div>

         <div class="inputs-list_item">
            <div class="input_title">
               <text  fill="#777" class="k_text" >k2</text>
               <text  fill="#777" class="text" >Шаг</text>
            </div>
            <input type="text" name="k2" id="k2" value="2" class="input_field"><br>
         </div>

         <div class="inputs-list_item">
            <div class="input_title">
               <text  fill="#777" class="k_text" >k3</text>
               <text  fill="#777" class="text" >Задержка</text>
            </div>
            <input type="text" name="k3" id="k3" value="3" class="input_field"><br>
         </div>

         <div class="inputs-list_item">
            <div class="input_title">
               <text  fill="#777" class="k_text" >k4</text>
               <text  fill="#777" class="text" >Световой порог на температуру</text>
            
            </div>
            <input type="text" name="k4" id="k4" value="4" class="input_field"><br>
         </div>

         <div class="inputs-list_item">
            <div class="input_title">
               <text  fill="#777" class="k_text" >k5</text>
               <text  fill="#777" class="text" >Зависимость от уличной t</text>
            
            </div>
           <input type="text" name="k5" id="k5" value="5" class="input_field"><br>
         </div>

         <div class="inputs-list_item">
            <div class="input_title">
               <text  fill="#777" class="k_text" >k6</text>
               <text  fill="#777" class="text" >Световой порог на вкл. фар (нету)</text>
            </div>
            <input type="text" name="k6" id="k6" value="6" class="input_field"><br>
         </div>

         <div class="inputs-list_item">
            <div class="input_title">
               <text  fill="#777" class="k_text" >k7</text>
               <text  fill="#777" class="text" >Зависимость от яркости 1/10000</text>
            </div>
            <input type="text" name="k7" id="k7" value="7" class="input_field"><br>
         </div>

         <div class="inputs-list_item">
            <div class="input_title">
               <text  fill="#777" class="k_text" >k8</text>
               <text  fill="#777" class="text" >Коэффициент "П" 1/10</text>
            </div>
            <input type="text" name="k8" id="k8" value="8" class="input_field"><br>
         </div>


         <div class="inputs-list_item">
            <div class="input_title">
               <text  fill="#777" class="text" >Диапазон сервомотора</text>
            </div>
            <input type="text" name="ServoMin" id="ServoMin" value="1500" class="input_field"><br>
            <vseparator></vseparator>
            <input type="text" name="ServoMax" id="ServoMax" value="2380" class="input_field"><br>
         </div>
         <div class="inputs-list_item">
            <div class="input_title">
               <text  fill="#777" class="text" >Диапазон резистора</text>
            </div>
            <input type="text" name="PotMin" id="PotMin" value="0" class="input_field"><br>
            <vseparator></vseparator>
            <input type="text" name="PotMax" id="PotMax" value="1024" class="input_field"><br>
         </div>
      </form>

      <separator></separator>
      

      <div class="button-row">
         <vseparator></vseparator>
         <button class="button-row_item button get-button text " id="myBtnGet">
            Get
         </button>  
         <button class="button-row_item button set-button text " id="myBtnSet">
            Set
         </button>  
         <vseparator></vseparator>
      </div>
      <div class="button-row">
         <vseparator></vseparator>
         <button class="button-row_item button load-button text " id="myBtnLoad">
            Load
         </button>
         <button class="button-row_item button save-button text " id="myBtnSave">
            Save
         </button>
         <vseparator></vseparator>
      </div>
      <separator></separator>
   </main>
</body>

<script>



function getData() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      console.log(this.responseText);
      var arr=this.responseText.split("|");
      document.getElementById("temp1").innerHTML=arr[0];
      document.getElementById("temp2").innerHTML=arr[4];
      document.getElementById("temp3").innerHTML=arr[5];
      document.getElementById("wanted_temp").innerHTML=arr[6];
      document.getElementById("wanted_temp2").innerHTML=arr[7];
      document.getElementById("pot_percent").innerHTML=arr[8];
      document.getElementById("servo_percent").innerHTML=arr[9];
      document.getElementById("light_level").innerHTML=arr[10];
    }
  };
  xhttp.open("GET", "getData", true);
  xhttp.send();
}

function setParameters() {
   let form = document.forms.k_form;
   let elem1 = form.elements.k1;
   let elem2 = form.elements.k2;
   let elem3 = form.elements.k3;
   let elem4 = form.elements.k4;
   let elem5 = form.elements.k5;
   let elem6 = form.elements.k6;
   let elem7 = form.elements.k7;
   let elem8 = form.elements.k8;

   let ServoMinEl = form.elements.ServoMin;
   let ServoMaxEl = form.elements.ServoMax;
   let PotMinEl = form.elements.PotMin;
   let PotMaxEl = form.elements.PotMax;

   console.log("Sending parameters:\n"+elem1.value+" "
   +elem2.value+" "
   +elem3.value+" "
   +elem4.value+" "
   +elem5.value+" "
   +elem6.value+" "
   +elem7.value+" "
   +elem8.value+" "
   +ServoMinEl.value+" "
   +ServoMaxEl.value+" "
   +PotMinEl.value+" "
   +PotMaxEl.value+" ");

   var xhttp = new XMLHttpRequest();
   xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
         console.log(this.responseText);
      }
      else{
         if (this.readyState == 4){
            alert("SetParameters returned with status "+this.status);
         }
      }
   };

   xhttp.timeout = 4000;
   xhttp.ontimeout = function () { alert("SetParameters Timed out"); }

   xhttp.open("GET", 
      "setParams?k1="+elem1.value
      +"&k2="+elem2.value
      +"&k3="+elem3.value
      +"&k4="+elem4.value
      +"&k5="+elem5.value
      +"&k6="+elem6.value
      +"&k7="+elem7.value
      +"&k8="+elem8.value
      +"&ServoMin="+ServoMinEl.value
      +"&ServoMax="+ServoMaxEl.value
      +"&PotMin="+PotMinEl.value
      +"&PotMax="+PotMaxEl.value
      , true);
   xhttp.send();
}


function getParameters() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      console.log(this.responseText);
      var arr=this.responseText.split("|");
      document.getElementById("k1").value=arr[0];
      document.getElementById("k2").value=arr[1];
      document.getElementById("k3").value=arr[2];
      document.getElementById("k4").value=arr[3];
      document.getElementById("k5").value=arr[4];
      document.getElementById("k6").value=arr[5];
      document.getElementById("k7").value=arr[6];
      document.getElementById("k8").value=arr[7];

      document.getElementById("ServoMin").value=arr[8];
      document.getElementById("ServoMax").value=arr[9];
      document.getElementById("PotMin").value=arr[10];
      document.getElementById("PotMax").value=arr[11];
      pageUpdate();
    }else{
      if (this.readyState == 4){
            alert("GetParameters returned with status "+this.status);
         }
    }
  };
  xhttp.timeout = 4000;
  xhttp.ontimeout = function () { alert("Get Parameters Timed out"); }
  xhttp.open("GET", "getParams", true);
  xhttp.send();
}

function pageUpdate(){
   getData();
   setTimeout(pageUpdate, 2000);
}


function loadParameters() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      getParameters();
    }else {
      if (this.readyState == 4){
            alert("LoadParameters returned with status "+this.status);
         }
    }
  };
  xhttp.timeout = 4000;
  xhttp.ontimeout = function () { alert("Load Parameters Timed out"); }
  xhttp.open("GET", "loadParams", true);
  xhttp.send();
}

function saveParameters(){
   let form = document.forms.k_form;
   let elem1 = form.elements.k1;
   let elem2 = form.elements.k2;
   let elem3 = form.elements.k3;
   let elem4 = form.elements.k4;
   let elem5 = form.elements.k5;
   let elem6 = form.elements.k6;

   let elem7 = form.elements.k7;
   let elem8 = form.elements.k8;

   let ServoMinEl = form.elements.ServoMin;
   let ServoMaxEl = form.elements.ServoMax;
   let PotMinEl = form.elements.PotMin;
   let PotMaxEl = form.elements.PotMax;

   console.log("Sending parameters to SAVE:\n"+elem1.value+" "
   +elem2.value+" "
   +elem3.value+" "
   +elem4.value+" "
   +elem5.value+" "
   +elem6.value+" "
   +elem7.value+" "
   +elem8.value+" "
   +ServoMinEl.value+" "
   +ServoMaxEl.value+" "
   +PotMinEl.value+" "
   +PotMaxEl.value+" ");

   var xhttp = new XMLHttpRequest();
   xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
         console.log(this.responseText);
      }
      else{
         if (this.readyState == 4){
            alert("SaveParameters returned with status "+this.status);
         }
      }
   };

   xhttp.timeout = 4000;
   xhttp.ontimeout = function () { alert("SaveParameters Timed out"); }

   xhttp.open("GET", 
      "saveParams?k1="+elem1.value
      +"&k2="+elem2.value
      +"&k3="+elem3.value
      +"&k4="+elem4.value
      +"&k5="+elem5.value
      +"&k6="+elem6.value
      +"&k7="+elem7.value
      +"&k8="+elem8.value
      +"&ServoMin="+ServoMinEl.value
      +"&ServoMax="+ServoMaxEl.value
      +"&PotMin="+PotMinEl.value
      +"&PotMax="+PotMaxEl.value
      , true);
   xhttp.send();
}

function GetBtnClick(){
   document.getElementById("myBtnGet").classList.add("clicked");
      setTimeout(()=>{
      document.getElementById("myBtnGet").classList.remove("clicked");
      }, 500);
      getParameters()
}

function SetBtnClick(){
   document.getElementById("myBtnSet").classList.add("clicked");
      setTimeout(()=>{
      document.getElementById("myBtnSet").classList.remove("clicked");
      }, 500);

   setParameters();
}

function LoadBtnClick(){
   document.getElementById("myBtnLoad").classList.add("clicked");
      setTimeout(()=>{
      document.getElementById("myBtnLoad").classList.remove("clicked");
      }, 500);

   loadParameters();
}

function SaveBtnClick(){
   document.getElementById("myBtnSave").classList.add("clicked");
      setTimeout(()=>{
      document.getElementById("myBtnSave").classList.remove("clicked");
      }, 500);

   saveParameters();
}

window.onload = (event) => {
  console.log('page is fully loaded');
  shouldUpdate=1;

  var GetBtn = document.getElementById("myBtnGet");
  GetBtn.addEventListener("click", GetBtnClick);
  GetBtn.addEventListener('touchstart', GetBtnClick);

  var SetBtn = document.getElementById("myBtnSet");
  SetBtn.addEventListener("click", SetBtnClick);
  SetBtn.addEventListener('touchstart', SetBtnClick);
  
  var LoadBtn = document.getElementById("myBtnLoad");
  LoadBtn.addEventListener("click", LoadBtnClick);
  LoadBtn.addEventListener('touchstart', LoadBtnClick);

  var SaveBtn = document.getElementById("myBtnSave");
  SaveBtn.addEventListener("click", SaveBtnClick);
  SaveBtn.addEventListener('touchstart', SaveBtnClick);

  pageUpdate();
  getParameters();
};


</script>
)=====";
