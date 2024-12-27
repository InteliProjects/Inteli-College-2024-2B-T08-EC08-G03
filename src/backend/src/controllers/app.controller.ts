import { Controller, Get } from '@nestjs/common';
import { AppService } from '@services/app.service';
import { testing, fodase} from '@services/test';

@Controller('/teste')
export class AppController {
  constructor(private readonly appService: AppService) { }

  @Get()
  getHello(): string {
    return this.appService.getHello();
  }
}

@Controller('/robot')
export class RobotController{
  constructor(private readonly appService: AppService){ }


  @Get()
  getRobots(): string {
    return fodase();
    //return this.appService.getRobots();
  }
}
