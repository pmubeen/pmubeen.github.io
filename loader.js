var myVar;

function myFunction() {
  myVar = setTimeout(showPage, 1000);
}

function showPage() { 
    var fade= document.getElementById("loader");
      
    var intervalID = setInterval(function () { 
          
        if (!fade.style.opacity) { 
            fade.style.opacity = 1; 
        } 
          
          
        if (fade.style.opacity > 0) { 
            fade.style.opacity -= 0.01; 
        }  
          
        else { 
            clearInterval(intervalID); 
            fade.style.display = "none";
            document.body.style.overflow = 'visible';
        } 
          
    }, 5);
}