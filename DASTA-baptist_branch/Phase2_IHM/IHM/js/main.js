
(function ($) {
    "use strict";

    // Spinner
    var spinner = function () {
        setTimeout(function () {
            if ($('#spinner').length > 0) {
                $('#spinner').removeClass('show');
            }
        }, 1);
    };
    spinner();

    // Back to top button
    $(window).scroll(function () {
        if ($(this).scrollTop() > 300) {
            $('.back-to-top').fadeIn('slow');
        } else {
            $('.back-to-top').fadeOut('slow');
        }
    });
    $('.back-to-top').click(function () {
        $('html, body').animate({scrollTop: 0}, 1500, 'easeInOutExpo');
        return false;
    });


    // Sidebar Toggler
    $('.sidebar-toggler').click(function () {
        $('.sidebar, .content').toggleClass("open");
        return false;
    });


    // Progress Bar
    $('.pg-bar').waypoint(function () {
        $('.progress .progress-bar').each(function () {
            $(this).css("width", $(this).attr("aria-valuenow") + '%');
        });
    }, {offset: '80%'});


    // Calender
    $('#calender').datetimepicker({
        inline: true,
        format: 'L'
    });


    // Testimonials carousel
    $(".testimonial-carousel").owlCarousel({
        autoplay: true,
        smartSpeed: 1000,
        items: 1,
        dots: true,
        loop: true,
        nav : false
    });


    // Chart Global Color
    Chart.defaults.color = "#6C7293";
    Chart.defaults.borderColor = "#000000";


    // Gyro Scopes
   var gyroXData = [];
   var gyroYData = [];
   var gyroZData = [];

   // kalman delay
    var kalman_delay1 = []
    var kalman_delay2 = []

    //close loop delay
    var close_loop_delay1 = []
    var close_loop_delay2 = []

    //stream delay
    var stream_delay1 = []
    var stream_delay2 = []

    // Motor power
    var motor1Data = [];
    var motor2Data = [];
    var motor3Data = [];
    var motor4Data = [];

    // Battery
    var batteryData1 = []
    var batteryData2 = []
    var batteryData3 = []

// Create a chart

var time =[]

// Fetch data from API and update the chart
function fetchDataFromAPI(url) {
    fetch(url)
        .then(response => response.json())
        .then(data => {
            // Add the latest gyro X data to the array
            gyroXData.push(data.data.gyro_raw[0]);
            gyroYData.push(data.data.gyro_raw[1]);
            gyroZData.push(data.data.gyro_raw[2]);

            //motors data
            motor1Data.push(data.data.motor_speeds[0]);
            motor2Data.push(data.data.motor_speeds[1]);
            motor3Data.push(data.data.motor_speeds[2]);
            motor4Data.push(data.data.motor_speeds[3]);

            //close loop delay
            close_loop_delay1.push(data.data.close_loop_delay_s[0]);
            close_loop_delay2.push(data.data.close_loop_delay_s[1]);

            //kalman delay
            kalman_delay1.push(data.data.kalman_delay_s[0]);
            kalman_delay2.push(data.data.kalman_delay_s[1]);
            
            //stream delay
            stream_delay1.push(data.data.stream_delay_s[0]);
            stream_delay2.push(data.data.stream_delay_s[1]);
             
            time.push((data.data.time_us/1000000).toFixed(1))
            // Ensure gyroXData only stores data for the last 10 seconds (or 10 data points)
            if (gyroXData.length > 10) {
                gyroXData.shift(); // Remove the oldest data point
                gyroYData.shift();
                gyroZData.shift();
                time.shift();
                motor1Data.shift();
                motor2Data.shift();
                motor3Data.shift();
                motor4Data.shift();
                kalman_delay1.shift();
                kalman_delay2.shift();

                close_loop_delay1.shift();
                close_loop_delay2.shift();
                
                stream_delay1.shift();
                stream_delay2.shift();
                
            }
            
        })
        .catch(error => console.error(error));
    }
    
    // Fetch data every 3 seconds
    setInterval(() => {
    myChart2.update();
    myChart1.update();
    myChart3.update();
    myChart7.update();
    myChart8.update();
    myChart9.update();
    fetchDataFromAPI("http://localhost:8000/recent_data/");
},1500);


const socket = new WebSocket('ws://localhost:8765');

const batteryTime = []
// Handle messages received from the server
socket.onmessage = function (event) {
  const data = JSON.parse(event.data);
     if (data.data?.battery_voltages){  
        console.log('Battery', data.data.battery_voltages);
                batteryData1.push(data.data.battery_voltages[0])
                batteryData2.push(data.data.battery_voltages[1])
                batteryData3.push(data.data.battery_voltages[2])
                batteryTime.push((data.data.time_us/1000000).toFixed(1))

                if (batteryData1.length > 10) {
                    batteryData1.shift(); // Remove the oldest data point
                    batteryData2.shift();
                    batteryData3.shift();
                    batteryTime.shift();
                }
            }

};

// Handle errors
socket.onerror = function (error) {
  console.error('WebSocket error:', error);
};

// Handle closure of the WebSocket connection
socket.onclose = function (event) {
  console.log('WebSocket connection closed:', event);
};


    // gyromter
    var ctx2 = $("#gyrometer").get(0).getContext("2d");
    var myChart2 = new Chart(ctx2, {
        type: "line",
        data: {
            labels: time,
            datasets: [{
                    label: "X-axis",
                    data: gyroXData,
                    backgroundColor: "rgba(22, 235, 22, .2)",
                    fill: true
                },
                {
                    label: "Y-axis",
                    data: gyroYData,
                    backgroundColor: "rgba(22, 22, 235, .2)",
                    fill: true
                },
                  {
                    label: "Z-axis",
                    data: gyroZData,
                    backgroundColor: "rgba(235, 22, 22, .2)",
                    fill: true
                }
            ]
            },
        options: {
            responsive: true
        }
    });

    //Kalman delay
    
    var ctx7 = $("#Kalman_delay").get(0).getContext("2d");
    var myChart7 = new Chart(ctx7, {
        type: "line",
        data: {
            labels: time,
            datasets: [{
                    label: "Last delay",
                    data: kalman_delay1,
                    backgroundColor: "rgba(22, 235, 22, .2)",
                    fill: true
                },
                {
                    label: "Max delay",
                    data: kalman_delay2,
                    backgroundColor: "rgba(22, 22, 235, .2)",
                    fill: true
                }
            ]
            },
        options: {
            responsive: true
        }
    });

    // close loop delay
    var ctx8 = $("#close_loop_delay").get(0).getContext("2d");
    var myChart8 = new Chart(ctx8, {
        type: "line",
        data: {
            labels: time,
            datasets: [{
                    label: "Last delay",
                    data: close_loop_delay1,
                    backgroundColor: "rgba(22, 235, 22, .2)",
                    fill: true
                },
                {
                    label: "Max delay",
                    data: close_loop_delay2,
                    backgroundColor: "rgba(235, 150, 35, .2)",
                    fill: true
                }
            ]
            },
        options: {
            responsive: true
        }
    });

      var ctx9 = $("#stream_delay_s").get(0).getContext("2d");
    var myChart9 = new Chart(ctx9, {
        type: "line",
        data: {
            labels: time,
            datasets: [{
                    label: "Last delay",
                    data: stream_delay1,
                    backgroundColor: "rgba(230, 35, 22, .2)",
                    fill: true
                },
                {
                    label: "Max delay",
                    data: stream_delay2,
                    backgroundColor: "rgba(35, 10, 35, .2)",
                    fill: true
                }
            ]
            },
        options: {
            responsive: true
        }
    });


    // BATTERIES
    var ctx3 = $("#battery").get(0).getContext("2d");
    var myChart3 = new Chart(ctx3, {
        type: "line",
        data: {
            labels: batteryTime,
           datasets: [{
                    label: "Battery 1",
                    data: batteryData1,
                    backgroundColor: "rgba(22, 235, 22, .2)",
                    fill: true
                },
                {
                    label: "Battery 2",
                    data: batteryData2,
                    backgroundColor: "rgba(22, 22, 235, .2)",
                    fill: true
                },
                  {
                    label: "Battery 3",
                    data: batteryData3,
                    backgroundColor: "rgba(235, 22, 22, .2)",
                    fill: true
                }
            ]
        },
        options: {
            responsive: true
        }
    });


 // MOTORS
    var ctx1 = $("#Motors").get(0).getContext("2d");
    var myChart1 = new Chart(ctx1, {
        type: "bar",
        data: {
            labels: time,
            datasets: [{
                    label: "Motor 1",
                    data: motor2Data,
                    backgroundColor: "rgba(235, 22, 22, .5)"
                },
                {
                    label: "Motor 2",
                    data: motor2Data,
                    backgroundColor: "rgba(22, 235, 22, .5)"
                },
                {
                    label: "Motor 3",
                    data: motor3Data,
                    backgroundColor: "rgba(22, 22, 235, .5)"
                },
                {
                    label: "Motor 4",
                    data: motor4Data,
                    backgroundColor: "rgba(50, 130, 22, .5)"
                },
            ]
            },
        options: {
            responsive: true
        }
    });


    // Single Bar Chart
    // var ctx4 = $("#bar-chart").get(0).getContext("2d");
    // var myChart4 = new Chart(ctx4, {
    //     type: "bar",
    //     data: {
    //         labels: ["Italy", "France", "Spain", "USA", "Argentina"],
    //         datasets: [{
    //             backgroundColor: [
    //                 "rgba(235, 22, 22, .7)",
    //                 "rgba(235, 22, 22, .6)",
    //                 "rgba(235, 22, 22, .5)",
    //                 "rgba(235, 22, 22, .4)",
    //                 "rgba(235, 22, 22, .3)"
    //             ],
    //             data: [55, 49, 44, 24, 15]
    //         }]
    //     },
    //     options: {
    //         responsive: true
    //     }
    // });


    // Pie Chart
    // var ctx5 = $("#pie-chart").get(0).getContext("2d");
    // var myChart5 = new Chart(ctx5, {
    //     type: "pie",
    //     data: {
    //         labels: ["Italy", "France", "Spain", "USA", "Argentina"],
    //         datasets: [{
    //             backgroundColor: [
    //                 "rgba(235, 22, 22, .7)",
    //                 "rgba(235, 22, 22, .6)",
    //                 "rgba(235, 22, 22, .5)",
    //                 "rgba(235, 22, 22, .4)",
    //                 "rgba(235, 22, 22, .3)"
    //             ],
    //             data: [55, 49, 44, 24, 15]
    //         }]
    //     },
    //     options: {
    //         responsive: true
    //     }
    // });


    // Doughnut Chart
    // var ctx6 = $("#doughnut-chart").get(0).getContext("2d");
    // var myChart6 = new Chart(ctx6, {
    //     type: "doughnut",
    //     data: {
    //         labels: ["Italy", "France", "Spain", "USA", "Argentina"],
    //         datasets: [{
    //             backgroundColor: [
    //                 "rgba(235, 22, 22, .7)",
    //                 "rgba(235, 22, 22, .6)",
    //                 "rgba(235, 22, 22, .5)",
    //                 "rgba(235, 22, 22, .4)",
    //                 "rgba(235, 22, 22, .3)"
    //             ],
    //             data: [55, 49, 44, 24, 15]
    //         }]
    //     },
    //     options: {
    //         responsive: true
    //     }
    // });

    
})(jQuery);

