import { Injectable, NotFoundException } from '@nestjs/common';
import { SupabaseService } from '@services/supabase.service';
import { Position } from '../interfaces/position.interface';
import axios from 'axios';

function removeAccents(text: string): string {
  return text.normalize("NFD").replace(/[\u0300-\u036f]/g, "");
}

@Injectable()
export class ActionService {
  constructor(private readonly supabaseService: SupabaseService) {}

  async handleAction(action: { name: string; parameters: any }): Promise<string> {
    console.log(action);
    switch (action.name) {
      case 'moveRobotToPlace':
        return this.moveRobotToPlace(action.parameters.places);
      default:
        return `No handler for action: ${action.name}`;
    }
  }


  private async moveRobot(positionsList: Position[]): Promise<string> {
    const apiUrl = `http://10.128.0.77:5000/move`;
    const response = await axios.post(apiUrl, positionsList);
    

    return `Robot moved to positions ${positionsList}. API Response: ${JSON.stringify(response.data)}`;
  }

  private async moveRobotToPlace(places: string[]): Promise<string> {
    const positions = await Promise.all(places.map(async place => {
      const position = await this.supabaseService.getPositionByName(removeAccents(place));
      return position[0];
    }));

    console.log(positions);

    const positions_mapped = positions.map(pos => {
      return {
        x: pos.x,
        y: pos.y,
        orientation: pos.orientation
      }
    });

    console.log(positions_mapped);
  
    return this.moveRobot(positions_mapped);
  }
}
